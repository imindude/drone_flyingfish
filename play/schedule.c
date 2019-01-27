/**
 * *********************************************************************************************************************
 *  __     __  __
 * |  |   |__ |__
 * |__| X |   |     FlyingFish
 *
 * *********************************************************************************************************************
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "schedule.h"
#include "linked_list.h"

/* ****************************************************************************************************************** */

typedef struct _IdleLocalData
{
    uint32_t    last_exe_s_;
    uint32_t    last_exe_ms_;
    uint32_t    idle_cnt_;
    int8_t      idle_rate_;
}
IdleLocalData;

/* ****************************************************************************************************************** */

static IdleLocalData    idle_data = {

        .last_exe_s_  = 0,
        .last_exe_ms_ = 0,
        .idle_cnt_    = 0,
        .idle_rate_   = 0
};

/* ****************************************************************************************************************** */

static void idle_thread(uint32_t now_us, void *param)
{
    IdleLocalData   *this = (IdleLocalData*)param;
    uint32_t        now_s = now_us / 1000000;
    uint32_t        now_ms = now_us / 1000;

    if (now_ms != this->last_exe_ms_)
    {
        this->last_exe_ms_ = now_ms;
        this->idle_cnt_++;

        if (this->last_exe_s_ != now_s)
        {
            this->idle_rate_  = this->idle_cnt_ * 100 / 1000;
            this->idle_cnt_   = 0;
            this->last_exe_s_ = now_s;
        }
    }
}

static bool idle_wakeup(uint32_t now_us, void *param)
{
    (void)now_us;
    (void)param;

    return true;
}

/* ****************************************************************************************************************** */

#define SCHEDULE_MAX_YIELD      3

typedef struct _LocalData
{
    ThreadFunc  thread_;
    WakeupFunc  wakeup_;
    void        *param_;
    int8_t      prio_;
    int8_t      dyn_prio_;
    int8_t      yield_cnt_;

    struct list_head    list_;
}
LocalData;

/* ****************************************************************************************************************** */

static struct   list_head   thread_list;

/* ****************************************************************************************************************** */

static void need_yield(LocalData *this)
{
    if ((this->prio_ != _SchedulePrio_SysIdle) && (++this->yield_cnt_ > SCHEDULE_MAX_YIELD))
    {
        this->dyn_prio_++;
        this->yield_cnt_ = 0;
    }
}

static void exec_thread(LocalData *this, uint32_t now_us)
{
    this->thread_(now_us, this->param_);
    this->dyn_prio_  = this->prio_;
    this->yield_cnt_ = 0;
}

void schedule_init(void)
{
    INIT_LIST_HEAD(&thread_list);

    schedule_signup(idle_thread, idle_wakeup, &idle_data, _SchedulePrio_SysIdle);
}

void schedule_signup(ThreadFunc thread, WakeupFunc wakeup, void *param, SchedulePrio prio)
{
    LocalData   *this = (LocalData*)calloc(1, sizeof(LocalData));

    this->thread_    = thread;
    this->wakeup_    = wakeup;
    this->param_     = param;
    this->prio_      = prio;
    this->dyn_prio_  = prio;
    this->yield_cnt_ = 0;

    list_add_tail(&this->list_, &thread_list);
}

void schedule_exec(void)
{
    LocalData   *select = NULL;
    LocalData   *choose, *tmp;
    uint32_t    now_us = sys_get_micros();

    list_for_each_entry_safe(choose, tmp, &thread_list, list_)
    {
        if (choose->wakeup_(now_us, choose->param_))
        {
            if (select == NULL)
            {
                select = choose;
            }
            else
            {
                if (choose->dyn_prio_ > select->dyn_prio_)
                {
                    need_yield(select);
                    select = choose;
                }
                else{
                    need_yield(choose);
                }
            }
        }
    }

    exec_thread(select, now_us);
}

/* end of file ****************************************************************************************************** */
