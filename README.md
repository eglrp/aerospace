# aerospace
2017年暑假空天比赛中用到的文件，有识别、测距以及控制部分



1. ctrl_vel_new_hat_finalv1_lcq.cpp这个文件是2017/07/10lcq帮我改写的ctrl_vel文件，里面是检测框框，然后根据HSV颜色检测轮廓


2. ctrl_vel_new_hat_finalv1_crx_waijieyuan.cpp 这个是我根据上面文件改的（目标头盔上加了圆片），检测框框后，检测圆片轮廓，获取外接圆圆心


3. ctrl_vel_new_hat_finalv1_lcq_average.cpp 这个是2017/07/11 lcq再帮我写的ctrl_vel文件，先将两张图像进行匹配，找到匹配点，然后根据检测框中的匹配点对，进行视差的计算。相当于第一个文件的减小误差版本。
