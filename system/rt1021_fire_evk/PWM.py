from machine import PWM
pwm=PWM(2,3,1000,100)
pwm.freq(2000)
pwm.duty(50)
pwm.deinit()
