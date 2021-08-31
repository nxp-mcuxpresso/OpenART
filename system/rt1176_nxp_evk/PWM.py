from machine import PWM
pwm=PWM(1,3,1000,100)
pwm.freq(2000)
pwm.duty()
pwm.deinit()
