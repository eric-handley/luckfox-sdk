// UVR SRAD Camera capture - RV1106G2 + Arducam IMX519 (4-lane MIPI)
//
// Self-contained capture: runs the rkaiq 3A engine in-process (no separate
// rkaiq_3A_server needed) and streams VI -> VENC over the RV1106 wrap/online
// (DVBM) path, writing a raw HEVC elementary stream to a file.
//
// The IMX519 reads out a 3072x1728 no-binning center crop and the RV1106 ISP
// main-path scaler downscales it 1.6:1 to 1920x1080 (supersample) for a
// sharper, properly anti-aliased 1080p output. The ISP input size is set
// via stIspOpt.stMaxSize; the channel stSize is the downscaled output.
//
// Distilled from the Rockchip simple_test samples
// (simple_vi_bind_venc_wrap_rv1106.c + simple_vi_bind_venc_rtsp.c) with every
// path we do not use removed (RTSP, OSD/RGN, IVS/IVA, TDE, JPEG, debreath,
// codec/format switching). Dedicated to our fixed 1080p H.265 pipeline.

#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <rk_aiq_user_api2_sysctl.h>
#include <rk_aiq_user_api2_ae.h>

#include "rk_debug.h"
#include "rk_defines.h"
#include "rk_mpi_mb.h"
#include "rk_mpi_sys.h"
#include "rk_mpi_venc.h"
#include "rk_mpi_vi.h"

// ---- our fixed setup ------------------------------------------------------
#define CAM_ID            0                 // rkisp main path
#define ISP_IN_WIDTH      3072              // sensor no-bin crop (ISP scaler input)
#define ISP_IN_HEIGHT     1728
#define VI_WIDTH          1920              // ISP downscaled output (VENC input)
#define VI_HEIGHT         1080
#define VI_BUF_COUNT      3
#define WRAP_LINE         (VI_HEIGHT / 4)   // ISP->VENC wrap buffer height
#define VENC_GOP          60
#define VENC_BITRATE_KB   (6 * 1024)        // 6 Mbps CBR
#define IQ_FILE_DIR       "/etc/iqfiles"
#define DEFAULT_OUT_PATH  "/data/uvr_capture.h265"

enum log_level { LOG_QUIET = 0, LOG_NORMAL = 1, LOG_VERBOSE = 2 };

static volatile bool g_quit = false;
static rk_aiq_sys_ctx_t *g_aiq_ctx = NULL;
static rk_aiq_working_mode_t g_aiq_mode = RK_AIQ_WORKING_MODE_NORMAL;
static VI_CHN_BUF_WRAP_S g_vi_wrap;
static enum log_level g_log_level = LOG_NORMAL;
static bool g_manual_ae = false;
static float g_manual_time = 0.001f;  // 1ms default (well under 16.67ms for 60fps)
static float g_manual_gain = 8.0f;    // moderate gain

static void sigterm_handler(int sig) {
    fprintf(stderr, "\nsignal %d, stopping\n", sig);
    g_quit = true;
}

static RK_U64 now_us(void) {
    struct timespec t = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (RK_U64)t.tv_sec * 1000000 + (RK_U64)t.tv_nsec / 1000;
}

// ---- rkaiq 3A (in-process, mandatory for the online ISP to stream) --------
static XCamReturn isp_sof_cb(rk_aiq_metas_t *meta) {
    (void)meta;
    return XCAM_RETURN_NO_ERROR;
}

static XCamReturn isp_err_cb(rk_aiq_err_msg_t *msg) {
    if (msg->err_code == XCAM_RETURN_BYPASS)
        g_quit = true;
    return XCAM_RETURN_NO_ERROR;
}

static int isp_start(void) {
    rk_aiq_static_info_t info;
    char hdr[16];

    setlinebuf(stdout);
    snprintf(hdr, sizeof(hdr), "%d", (int)g_aiq_mode);
    setenv("HDR_MODE", hdr, 1); // must be set before init

    if (rk_aiq_uapi2_sysctl_enumStaticMetas(CAM_ID, &info)) {
        printf("ERROR: enumStaticMetas failed\n");
        return -1;
    }
    printf("3A: cam %d sensor '%s' iq '%s'\n", CAM_ID,
           info.sensor_info.sensor_name, IQ_FILE_DIR);

    g_aiq_ctx = rk_aiq_uapi2_sysctl_init(info.sensor_info.sensor_name,
                                         IQ_FILE_DIR, isp_err_cb, isp_sof_cb);
    if (!g_aiq_ctx) {
        printf("ERROR: rkaiq sysctl_init failed\n");
        return -1;
    }
    if (rk_aiq_uapi2_sysctl_prepare(g_aiq_ctx, 0, 0, g_aiq_mode)) {
        printf("ERROR: rkaiq prepare failed\n");
        return -1;
    }
    if (rk_aiq_uapi2_sysctl_start(g_aiq_ctx)) {
        printf("ERROR: rkaiq start failed\n");
        return -1;
    }
    printf("3A: rkaiq running\n");

    // Let the IQ file's AecFrameRateMode (isFpsFix:1, FpsValue:30) drive the
    // cadence, exactly like rkaiq_3A_server does (no setExpSwAttr). The old
    // in-process runtime fps override re-clamped output to 15fps even with a
    // 30fps IQ, so it is disabled. Kept (commented) in case manual AE needs it:
    //     expSwAttr.stAuto.stFrmRate.isFpsFix = true;
    //     expSwAttr.stAuto.stFrmRate.FpsValue = 30;
    if (g_manual_ae) {
        Uapi_ExpSwAttrV2_t expSwAttr;
        memset(&expSwAttr, 0, sizeof(expSwAttr));
        rk_aiq_user_api2_ae_getExpSwAttr(g_aiq_ctx, &expSwAttr);
        expSwAttr.AecOpType = RK_AIQ_OP_MODE_MANUAL;
        expSwAttr.stManual.LinearAE.ManualTimeEn = true;
        expSwAttr.stManual.LinearAE.ManualGainEn = true;
        expSwAttr.stManual.LinearAE.TimeValue = g_manual_time;
        expSwAttr.stManual.LinearAE.GainValue = g_manual_gain;
        XCamReturn ae_ret = rk_aiq_user_api2_ae_setExpSwAttr(g_aiq_ctx, expSwAttr);
        printf("3A: manual AE time=%.4f gain=%.1f (ret=%d)\n",
               g_manual_time, g_manual_gain, ae_ret);
    }

    return 0;
}

static void isp_stop(void) {
    if (!g_aiq_ctx)
        return;
    rk_aiq_uapi2_sysctl_stop(g_aiq_ctx, false);
    rk_aiq_uapi2_sysctl_deinit(g_aiq_ctx);
    g_aiq_ctx = NULL;
}

// ---- VI -------------------------------------------------------------------
static int vi_dev_init(void) {
    VI_DEV_ATTR_S stDevAttr;
    VI_DEV_BIND_PIPE_S stBindPipe;
    int ret;

    memset(&stDevAttr, 0, sizeof(stDevAttr));
    memset(&stBindPipe, 0, sizeof(stBindPipe));

    if (RK_MPI_VI_GetDevAttr(0, &stDevAttr) == RK_ERR_VI_NOT_CONFIG) {
        ret = RK_MPI_VI_SetDevAttr(0, &stDevAttr);
        if (ret != RK_SUCCESS) {
            printf("ERROR: VI SetDevAttr %x\n", ret);
            return -1;
        }
    }
    if (RK_MPI_VI_GetDevIsEnable(0) != RK_SUCCESS) {
        ret = RK_MPI_VI_EnableDev(0);
        if (ret != RK_SUCCESS) {
            printf("ERROR: VI EnableDev %x\n", ret);
            return -1;
        }
        stBindPipe.u32Num = 1;
        stBindPipe.PipeId[0] = 0;
        ret = RK_MPI_VI_SetDevBindPipe(0, &stBindPipe);
        if (ret != RK_SUCCESS) {
            printf("ERROR: VI SetDevBindPipe %x\n", ret);
            return -1;
        }
    }
    return 0;
}

static int vi_chn_init(void) {
    VI_CHN_ATTR_S attr;
    int ret;

    memset(&attr, 0, sizeof(attr));
    attr.stIspOpt.u32BufCount = VI_BUF_COUNT;
    attr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
    attr.stSize.u32Width = VI_WIDTH;
    attr.stSize.u32Height = VI_HEIGHT;
    attr.enPixelFormat = RK_FMT_YUV420SP;
    // stMaxSize is the ISP input (sensor) resolution; stSize is the scaled
    // output. The rkisp main-path scaler downscales 3072x1728 -> 1920x1080.
    attr.stIspOpt.stMaxSize.u32Width = ISP_IN_WIDTH;
    attr.stIspOpt.stMaxSize.u32Height = ISP_IN_HEIGHT;
    // depth must be >=1 (and < u32BufCount) when binding to VENC, otherwise
    // GetStream blocks forever; framerate -1 = inherit sensor cadence.
    attr.u32Depth = 1;
    attr.stFrameRate.s32SrcFrameRate = -1;
    attr.stFrameRate.s32DstFrameRate = -1;

    ret = RK_MPI_VI_SetChnAttr(0, CAM_ID, &attr);
    if (ret) {
        printf("ERROR: VI SetChnAttr %d\n", ret);
        return ret;
    }

    // OFFLINE mode: wrap DISABLED. The RV1106 ISP->VENC wrap/online (DVBM)
    // path halves a genuine 30fps ISP stream to 15fps (the encoder receives
    // 30fps but emits every other frame) - reproduced in Rockchip's own
    // simple_vi_bind_venc_wrap sample. Offline (full frames via memory) comes
    // up "online 0" and runs the full 30fps. Old wrap setup kept for ref:
    //   memset(&g_vi_wrap, 0, sizeof(g_vi_wrap));
    //   g_vi_wrap.bEnable = RK_TRUE;
    //   g_vi_wrap.u32BufLine = WRAP_LINE;
    //   g_vi_wrap.u32WrapBufferSize = WRAP_LINE * VI_WIDTH * 3 / 2; // nv12
    //   RK_MPI_VI_SetChnWrapBufAttr(0, CAM_ID, &g_vi_wrap);

    ret = RK_MPI_VI_EnableChn(0, CAM_ID);
    if (ret) {
        printf("ERROR: VI EnableChn %d\n", ret);
        return ret;
    }
    return 0;
}

// ---- VENC -----------------------------------------------------------------
static int venc_init(void) {
    VENC_CHN_ATTR_S attr;
    // VENC_CHN_BUF_WRAP_S wrap;  // OFFLINE mode: wrap disabled
    VENC_RECV_PIC_PARAM_S recv;

    memset(&attr, 0, sizeof(attr));
    attr.stVencAttr.enType = RK_VIDEO_ID_HEVC;
    attr.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
    attr.stVencAttr.u32MaxPicWidth = VI_WIDTH;
    attr.stVencAttr.u32MaxPicHeight = VI_HEIGHT;
    attr.stVencAttr.u32PicWidth = VI_WIDTH;
    attr.stVencAttr.u32PicHeight = VI_HEIGHT;
    attr.stVencAttr.u32VirWidth = VI_WIDTH;
    attr.stVencAttr.u32VirHeight = VI_HEIGHT;
    attr.stVencAttr.u32StreamBufCnt = 5;
    attr.stVencAttr.u32BufSize = VI_WIDTH * VI_HEIGHT * 3 / 2;
    attr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
    attr.stRcAttr.stH265Cbr.u32SrcFrameRateNum = 30;
    attr.stRcAttr.stH265Cbr.u32SrcFrameRateDen = 1;
    attr.stRcAttr.stH265Cbr.fr32DstFrameRateNum = 30;
    attr.stRcAttr.stH265Cbr.fr32DstFrameRateDen = 1;
    attr.stRcAttr.stH265Cbr.u32BitRate = VENC_BITRATE_KB;
    attr.stRcAttr.stH265Cbr.u32Gop = VENC_GOP;
    attr.stGopAttr.u32MaxLtrCount = 1;

    if (RK_MPI_VENC_CreateChn(0, &attr) != RK_SUCCESS) {
        printf("ERROR: VENC CreateChn failed\n");
        return -1;
    }

    // OFFLINE mode: VENC wrap DISABLED (see vi_chn_init). Old wrap setup:
    //   memset(&wrap, 0, sizeof(wrap));
    //   wrap.bEnable = g_vi_wrap.bEnable;
    //   wrap.u32BufLine = WRAP_LINE;
    //   RK_MPI_VENC_SetChnBufWrapAttr(0, &wrap);

    memset(&recv, 0, sizeof(recv));
    recv.s32RecvPicNum = -1; // unlimited; loop count gates stop instead
    RK_MPI_VENC_StartRecvFrame(0, &recv);
    return 0;
}

// Diagnostic: dump raw NV12 frames straight off the VI channel, bypassing
// VENC entirely. Isolates ISP-stage artifacts (scaler/demosaic) from encoder
// artifacts. View with e.g. ffplay -f rawvideo -pix_fmt nv12 -video_size WxH
// using the stride (VirWidth x VirHeight) printed per frame.
static int vi_dump_raw(const char *path, RK_S32 nframes) {
    FILE *fp = fopen(path, "wb");
    if (!fp) {
        printf("ERROR: cannot open %s\n", path);
        return -1;
    }
    VIDEO_FRAME_INFO_S vframe;
    RK_S32 got = 0;
    // Discard ~2s of frames so AF/AE converge before we keep one; the first
    // frames are always soft/mis-exposed and useless for judging detail.
    RK_S32 warmup = 60;
    while (!g_quit && warmup > 0) {
        if (RK_MPI_VI_GetChnFrame(0, CAM_ID, &vframe, 1000) != RK_SUCCESS)
            continue;
        RK_MPI_VI_ReleaseChnFrame(0, CAM_ID, &vframe);
        warmup--;
    }
    while (!g_quit && (nframes < 0 || got < nframes)) {
        if (RK_MPI_VI_GetChnFrame(0, CAM_ID, &vframe, 1000) != RK_SUCCESS)
            continue;
        void *data = RK_MPI_MB_Handle2VirAddr(vframe.stVFrame.pMbBlk);
        RK_U32 w = vframe.stVFrame.u32Width;
        RK_U32 h = vframe.stVFrame.u32Height;
        RK_U32 vw = vframe.stVFrame.u32VirWidth;
        RK_U32 vh = vframe.stVFrame.u32VirHeight;
        size_t sz = (size_t)vw * vh * 3 / 2; // NV12 incl. stride padding
        fwrite(data, 1, sz, fp);
        printf("raw frame %d: %ux%u (stride %ux%u) NV12 %zu bytes\n",
               got, w, h, vw, vh, sz);
        RK_MPI_VI_ReleaseChnFrame(0, CAM_ID, &vframe);
        got++;
    }
    fclose(fp);
    printf("dumped %d NV12 frame(s) to %s\n", got, path);
    return 0;
}

int main(int argc, char *argv[]) {
    const char *out_path = DEFAULT_OUT_PATH;
    const char *raw_path = NULL;
    RK_S32 frame_cnt = -1; // -1 = until SIGINT
    int ret = -1;
    int c;

    while ((c = getopt(argc, argv, "l:o:r:m::g:qvh")) != -1) {
        switch (c) {
        case 'l':
            frame_cnt = atoi(optarg);
            break;
        case 'o':
            out_path = optarg;
            break;
        case 'r':
            raw_path = optarg;
            break;
        case 'm':
            g_manual_ae = true;
            if (optarg)
                g_manual_time = strtof(optarg, NULL);
            break;
        case 'g':
            g_manual_gain = strtof(optarg, NULL);
            break;
        case 'q':
            g_log_level = LOG_QUIET;
            break;
        case 'v':
            g_log_level = LOG_VERBOSE;
            break;
        case 'h':
        default:
            printf("usage: %s [-l frames] [-o out.h265] [-r raw.nv12] [-m[time]] [-g gain] [-q|-v]\n", argv[0]);
            printf("  -l  number of frames to capture (default: until Ctrl-C)\n");
            printf("  -o  output raw HEVC path (default: %s)\n", DEFAULT_OUT_PATH);
            printf("  -r  dump raw NV12 from VI (bypass VENC) to path, then exit\n");
            printf("  -m  manual AE with optional exposure time in seconds (default: %.4f)\n", g_manual_time);
            printf("  -g  manual AE gain (default: %.1f)\n", g_manual_gain);
            printf("  -q  quiet: only errors and summary\n");
            printf("  -v  verbose: per-frame output\n");
            return c == 'h' ? 0 : -1;
        }
    }

    printf("UVR capture: %dx%d H.265, frames=%d, out=%s\n", VI_WIDTH, VI_HEIGHT,
           frame_cnt, out_path);

    signal(SIGINT, sigterm_handler);
    signal(SIGTERM, sigterm_handler);

    FILE *fp = NULL;
    if (!raw_path) {
        fp = fopen(out_path, "wb");
        if (!fp) {
            printf("ERROR: cannot open %s\n", out_path);
            return -1;
        }
        // Batch into large sequential writes: gentler on the SD FTL than the
        // default ~4KB dribble, and fewer write() syscalls on this single core.
        // ~1.7s of footage rides in RAM, lost only on an unexpected power cut.
        static char io_buf[2 * 1024 * 1024];
        setvbuf(fp, io_buf, _IOFBF, sizeof(io_buf));
    }

    if (isp_start() != 0)
        goto cleanup_file;

    if (RK_MPI_SYS_Init() != RK_SUCCESS) {
        printf("ERROR: RK_MPI_SYS_Init failed\n");
        goto cleanup_isp;
    }

    if (vi_dev_init() != 0 || vi_chn_init() != 0)
        goto cleanup_sys;

    if (raw_path) {
        vi_dump_raw(raw_path, frame_cnt < 0 ? 1 : frame_cnt);
        ret = 0;
        goto cleanup_vi;
    }

    if (venc_init() != 0)
        goto cleanup_vi;

    MPP_CHN_S src = {.enModId = RK_ID_VI, .s32DevId = 0, .s32ChnId = CAM_ID};
    MPP_CHN_S dst = {.enModId = RK_ID_VENC, .s32DevId = 0, .s32ChnId = 0};
    if (RK_MPI_SYS_Bind(&src, &dst) != RK_SUCCESS) {
        printf("ERROR: bind VI->VENC failed\n");
        goto cleanup_venc;
    }

    VENC_STREAM_S frame;
    frame.pstPack = malloc(sizeof(VENC_PACK_S));

    RK_S32 count = 0;
    RK_U64 first_pts = 0, last_pts = 0;
    RK_U64 total_bytes = 0;
    while (!g_quit) {
        if (RK_MPI_VENC_GetStream(0, &frame, -1) == RK_SUCCESS) {
            void *data = RK_MPI_MB_Handle2VirAddr(frame.pstPack->pMbBlk);
            fwrite(data, 1, frame.pstPack->u32Len, fp);

            RK_U64 pts = frame.pstPack->u64PTS;
            if (count == 0) first_pts = pts;
            last_pts = pts;
            total_bytes += frame.pstPack->u32Len;

            if (g_log_level >= LOG_VERBOSE) {
                printf("frame %d seq:%d len:%u pts=%lld delay=%lldus\n", count,
                       frame.u32Seq, frame.pstPack->u32Len,
                       (long long)pts, (long long)(now_us() - pts));
            } else if (g_log_level >= LOG_NORMAL && (count % 30 == 0)) {
                printf("frame %d seq:%d delay=%lldus\n", count,
                       frame.u32Seq, (long long)(now_us() - pts));
            }

            RK_MPI_VENC_ReleaseStream(0, &frame);
            count++;

            if (count == 10) {
                Uapi_ExpQueryInfo_t expInfo;
                memset(&expInfo, 0, sizeof(expInfo));
                if (rk_aiq_user_api2_ae_queryExpResInfo(g_aiq_ctx, &expInfo) == 0) {
                    printf("AE: fps=%.1f VTS=%.0f HTS=%.0f pixclk=%.2fMHz "
                           "time=%.6fs gain=%.1f converged=%d\n",
                           expInfo.Fps,
                           expInfo.LinePeriodsPerField,
                           expInfo.PixelPeriodsPerLine,
                           expInfo.PixelClockFreqMHZ,
                           expInfo.LinAeInfo.LinearExp.integration_time,
                           expInfo.LinAeInfo.LinearExp.analog_gain,
                           expInfo.IsConverged);
                }
            }

            if (frame_cnt >= 0 && count >= frame_cnt)
                break;
        }
    }
    fflush(fp);

    if (count > 1 && last_pts > first_pts) {
        double dur_s = (last_pts - first_pts) / 1e6;
        printf("captured %d frames in %.2fs (%.1f fps), %.1f KB total\n",
               count, dur_s, (count - 1) / dur_s, total_bytes / 1024.0);
    } else {
        printf("captured %d frames, %llu bytes\n", count,
               (unsigned long long)total_bytes);
    }

    free(frame.pstPack);
    ret = 0;

    RK_MPI_SYS_UnBind(&src, &dst);
cleanup_venc:
    RK_MPI_VENC_StopRecvFrame(0);
    RK_MPI_VENC_DestroyChn(0);
cleanup_vi:
    RK_MPI_VI_DisableChn(0, CAM_ID);
    RK_MPI_VI_DisableDev(0);
cleanup_sys:
    RK_MPI_SYS_Exit();
cleanup_isp:
    isp_stop();
cleanup_file:
    if (fp)
        fclose(fp);
    printf("UVR capture exit: %d (%s)\n", ret, raw_path ? raw_path : out_path);
    return ret;
}
