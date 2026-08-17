/**
 * @file    bms_balance_rtt.h
 * @brief   均衡监控 RTT 打印（1 Hz，state/mask 变化即打）
 */
#ifndef BMS_BALANCE_RTT_H
#define BMS_BALANCE_RTT_H

#ifdef __cplusplus
extern "C" {
#endif

void BmsBalanceRtt_Init(void);
void BmsBalanceRtt_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* BMS_BALANCE_RTT_H */
