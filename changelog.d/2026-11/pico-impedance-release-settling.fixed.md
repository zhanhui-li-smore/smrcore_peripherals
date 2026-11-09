<!-- release-category: pico -->

## 中文

Pico 阻抗遥操松开 Grip 后，对控制器返回的 4311 阻尼停稳超时最多等待三轮退出调用，避免首次 2 秒等待超时就触发急停。等待期间不再下发目标或重新进入阻抗；持续超时、其他退出错误、断连、锁存错误或控制权丢失仍进入急停兜底。保留关节速度保护。

## English

Pico impedance teleoperation now allows up to three exit calls when the controller reports 4311 while damping to rest, instead of triggering EStop after the first two-second settling window. No targets or reactivation are allowed during exit. Persistent timeout, other exit errors, disconnection, latched faults or lost ownership retain the EStop fallback. Joint velocity protection is unchanged.
