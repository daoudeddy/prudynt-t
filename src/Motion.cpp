#include "Motion.hpp"

using namespace std::chrono;
bool ignoreInitialPeriod = true;

std::string Motion::getConfigPath(const char *itemName)
{
    return "motion." + std::string(itemName);
}

void Motion::detect()
{
    LOG_INFO("Start motion detection thread.");

    int ret;
    int debounce = 0;
    // IMP_IVS_MoveOutput *result;
    persondet_param_output_t *result;

    bool isInCooldown = false;
    auto cooldownEndTime = steady_clock::now();
    auto motionEndTime = steady_clock::now();
    auto startTime = steady_clock::now();

    if (init() != 0)
        return;

    global_motion_thread_signal = true;
    while (global_motion_thread_signal)
    {

        ret = IMP_IVS_PollingResult(ivsChn, cfg->motion.ivs_polling_timeout);
        if (ret < 0)
        {
            LOG_WARN("IMP_IVS_PollingResult error: " << ret);
            continue;
        }

        ret = IMP_IVS_GetResult(ivsChn, (void **)&result);
        if (ret < 0)
        {
            LOG_WARN("IMP_IVS_GetResult error: " << ret);
            continue;
        }

        auto currentTime = steady_clock::now();
        auto elapsedTime = duration_cast<seconds>(currentTime - startTime);

        if (ignoreInitialPeriod && elapsedTime.count() < cfg->motion.init_time)
        {
            continue;
        }
        else
        {
            ignoreInitialPeriod = false;
        }

        if (isInCooldown && duration_cast<seconds>(currentTime - cooldownEndTime).count() < cfg->motion.cooldown_time)
        {
            continue;
        }
        else
        {
            isInCooldown = false;
        }

        bool motionDetected = false;
        persondet_param_output_t *r = (persondet_param_output_t *)result;
        for (int j = 0; j < r->count; j++)
        {
            motionDetected = true;
            LOG_INFO("Active motion detected in region " << j);
            debounce++;
            if (debounce >= cfg->motion.debounce_time)
            {
                if (!moving.load())
                {
                    moving = true;
                    LOG_INFO("Motion Start");

                    IVSRect *rect = &r->person[j].show_box;

                    char cmd[128];
                    memset(cmd, 0, sizeof(cmd));
                    snprintf(cmd, sizeof(cmd), "%s start", cfg->motion.script_path);
                    ret = system(cmd);

                    if (ret != 0)
                    {
                        LOG_ERROR("Motion script failed:" << cmd);
                    }
                    // printf("person location:%d, %d, %d, %d\n", rect->ul.x, rect->ul.y, rect->br.x, rect->br.y);
                    // printf("person confidence:%f\n", r->person[j].confidence);
                    LOG_INFO("Person detected:" << " left x: " << rect->ul.x << ", left y:" << rect->ul.y << ", right x:" << rect->br.x << ", right y:" << rect->br.y << ", confidence:" << (r->person[j].confidence * 100));
                }
                indicator = true;
                motionEndTime = steady_clock::now(); // Update last motion time
            }
        }

        if (!motionDetected)
        {
            debounce = 0;
            auto duration = duration_cast<seconds>(currentTime - motionEndTime).count();
            if (moving && duration >= cfg->motion.min_time && duration >= cfg->motion.post_time)
            {
                LOG_INFO("End of Motion");
                char cmd[128];
                memset(cmd, 0, sizeof(cmd));
                snprintf(cmd, sizeof(cmd), "%s stop", cfg->motion.script_path);
                ret = system(cmd);
                if (ret != 0)
                {
                    LOG_ERROR("Motion script failed:" << cmd);
                }
                moving = false;
                indicator = false;
                cooldownEndTime = steady_clock::now(); // Start cooldown
                isInCooldown = true;
            }
        }

        ret = IMP_IVS_ReleaseResult(ivsChn, (void *)result);
        if (ret < 0)
        {
            LOG_WARN("IMP_IVS_ReleaseResult error: " << ret);
            continue;
        }
    }

    exit();

    LOG_DEBUG("Exit motion detect thread.");
}

int Motion::init()
{
    LOG_INFO("Initialize motion detection.");

    if ((cfg->motion.monitor_stream == 0 && !cfg->stream0.enabled) ||
        (cfg->motion.monitor_stream == 1 && !cfg->stream1.enabled))
    {

        LOG_ERROR("Monitor stream is disabled, abort.");
        return -1;
    }
    int ret;

    ret = IMP_IVS_CreateGroup(0);
    LOG_DEBUG_OR_ERROR_AND_EXIT(ret, "IMP_IVS_CreateGroup(0)");

    // automatically set frame size / height
    ret = IMP_Encoder_GetChnAttr(cfg->motion.monitor_stream, &channelAttributes);
    if (ret == 0)
    {
        if (cfg->motion.frame_width == IVS_AUTO_VALUE)
        {
            cfg->set<int>(getConfigPath("frame_width"), channelAttributes.encAttr.picWidth, true);
        }
        if (cfg->motion.frame_height == IVS_AUTO_VALUE)
        {
            cfg->set<int>(getConfigPath("frame_height"), channelAttributes.encAttr.picHeight, true);
        }
        if (cfg->motion.roi_1_x == IVS_AUTO_VALUE)
        {
            cfg->set<int>(getConfigPath("roi_1_x"), channelAttributes.encAttr.picWidth - 1, true);
        }
        if (cfg->motion.roi_1_y == IVS_AUTO_VALUE)
        {
            cfg->set<int>(getConfigPath("roi_1_y"), channelAttributes.encAttr.picHeight - 1, true);
        }
    }

    memset(&person_param, 0, sizeof(persondet_param_input_t));
    // OSD is affecting motion for some reason.
    // Sensitivity range is 0-4
    person_param.frameInfo.width = cfg->motion.frame_width; // 800;
    person_param.frameInfo.height = cfg->motion.frame_height;

    person_param.skip_num = cfg->motion.skip_frame_count; // skip num
    person_param.ptime = false;                           // print time or not
    person_param.sense = cfg->motion.sensitivity;         // detection sensibility
    person_param.detdist = 2;                             // detection distance
    person_param.enable_perm = false;
    person_param.enable_move = false;
    person_param.permcnt = 2;
    person_param.perms[0].pcnt = 6;
    person_param.perms[1].pcnt = 5;
    person_param.perms[0].p = (IVSPoint *)malloc(12 * sizeof(int));
    person_param.perms[1].p = (IVSPoint *)malloc(12 * sizeof(int));
    int i, j;
    for (i = 0; i < person_param.permcnt; i++)
    {
        switch (i)
        {
        case 0:
        {
            for (j = 0; j < person_param.perms[0].pcnt; j++)
            {
                person_param.perms[0].p[j].x = (j % 3) * 140;
                person_param.perms[0].p[j].y = (j / 3) * 340;
            }
        }
        break;
        case 1:
        {
            for (j = 0; j < person_param.perms[1].pcnt; j++)
            {
                person_param.perms[1].p[j].x = (j % 3) * 140 + 321;
                person_param.perms[1].p[j].y = (j / 3) * 340;
            }
        }
        break;
        }
    }
    person_intf = PersonDetInterfaceInit(&person_param);

    // move_param.sense[0] = cfg->motion.sensitivity;
    // move_param.skipFrameCnt = cfg->motion.skip_frame_count;
    // move_param.frameInfo.width = cfg->motion.frame_width;
    // move_param.frameInfo.height = cfg->motion.frame_height;

    // LOG_INFO("Motion detection:" << " sensibility: " << move_param.sense[0] << ", skipCnt:" << move_param.skipFrameCnt << ", width:" << move_param.frameInfo.width << ", height:" << move_param.frameInfo.height);

    // move_param.roiRect[0].p0.x = cfg->motion.roi_0_x;
    // move_param.roiRect[0].p0.y = cfg->motion.roi_0_y;
    // move_param.roiRect[0].p1.x = cfg->motion.roi_1_x - 1;
    // move_param.roiRect[0].p1.y = cfg->motion.roi_1_y - 1;
    // move_param.roiRectCnt = cfg->motion.roi_count;

    // LOG_INFO("Motion detection roi[0]:" << " roi_0_x: " << cfg->motion.roi_0_x << ", roi_0_y:" << cfg->motion.roi_0_y << ", roi_1_x: " << cfg->motion.roi_1_x << ", roi_1_y:" << cfg->motion.roi_1_y);

    // move_intf = IMP_IVS_CreateMoveInterface(&move_param);

    ret = IMP_IVS_CreateChn(ivsChn, person_intf);
    LOG_DEBUG_OR_ERROR_AND_EXIT(ret, "IMP_IVS_CreateChn(" << ivsChn << ", move_intf)");

    ret = IMP_IVS_RegisterChn(ivsGrp, ivsChn);
    LOG_DEBUG_OR_ERROR_AND_EXIT(ret, "IMP_IVS_RegisterChn(" << ivsGrp << ", " << ivsChn << ")");

    ret = IMP_IVS_StartRecvPic(ivsChn);
    LOG_DEBUG_OR_ERROR_AND_EXIT(ret, "IMP_IVS_StartRecvPic(" << ivsChn << ")")

    fs = {
        /**< Device ID */ DEV_ID_FS,
        /**< Group ID */ cfg->motion.monitor_stream,
        /**< output ID */ 1};

    ivs_cell = {
        /**< Device ID */ DEV_ID_IVS,
        /**< Group ID */ 0,
        /**< output ID */ 0};

    ret = IMP_System_Bind(&fs, &ivs_cell);
    LOG_DEBUG_OR_ERROR_AND_EXIT(ret, "IMP_System_Bind(&fs, &ivs_cell)");

    return ret;
}

int Motion::exit()
{
    int ret;

    LOG_DEBUG("Exit motion detection.");

    ret = IMP_IVS_StopRecvPic(ivsChn);
    LOG_DEBUG_OR_ERROR(ret, "IMP_IVS_StopRecvPic(0)");

    ret = IMP_System_UnBind(&fs, &ivs_cell);
    LOG_DEBUG_OR_ERROR_AND_EXIT(ret, "IMP_System_UnBind(&fs, &ivs_cell)");

    ret = IMP_IVS_UnRegisterChn(ivsChn);
    LOG_DEBUG_OR_ERROR(ret, "IMP_IVS_UnRegisterChn(0)");

    ret = IMP_IVS_DestroyChn(ivsChn);
    LOG_DEBUG_OR_ERROR(ret, "IMP_IVS_DestroyChn(0)");

    ret = IMP_IVS_DestroyGroup(ivsGrp);
    LOG_DEBUG_OR_ERROR(ret, "IMP_IVS_DestroyGroup(0)");

    PersonDetInterfaceExit(person_intf);

    return ret;
}

void *Motion::run(void *arg)
{
    ((Motion *)arg)->detect();
    return nullptr;
}
