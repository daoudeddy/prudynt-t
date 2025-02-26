#ifndef Motion_hpp
#define Motion_hpp

#include <memory>
#include <thread>
#include <atomic>
#include "Config.hpp"
#include "Logger.hpp"
#include "globals.hpp"
#include "imp/imp_system.h"
#include "imp/imp_ivs.h"
#include "imp/imp_ivs_move.h"

#include <ivs/ivs_common.h>
#include <ivs/ivs_interface.h>
#include <ivs/ivs_inf_personDet.h>
#include <imp/imp_ivs.h>

#define NR_FRAMES_TO_IVS		200

#if defined(PLATFORM_T31)
#define IMPEncoderCHNAttr IMPEncoderChnAttr
#define IMPEncoderCHNStat IMPEncoderChnStat
#endif

#if defined(PLATFORM_T31)
#define picWidth uWidth
#define picHeight uHeight
#endif

class Motion {
    public:
        void detect();
        static void *run(void* arg);
        int init();
        int exit();

    private:
        int ivsChn = 0;
        int ivsGrp = 0;

        std::string getConfigPath(const char *itemName);

        std::atomic<bool> moving;
        std::atomic<bool> indicator;    
        // IMP_IVS_MoveParam move_param;
        persondet_param_input_t person_param;
        // IMPIVSInterface *move_intf;
        IMPIVSInterface *person_intf;

        std::thread detect_thread;

        IMPCell fs = {};
        IMPCell ivs_cell = {};

        IMPEncoderCHNAttr channelAttributes;
};

#endif /* Motion_hpp */
