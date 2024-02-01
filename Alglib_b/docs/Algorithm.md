# Algorithm

路口场景各模块算法选择参数，对应 isfp_baseline/OutPut/Configs/Alg/Alg.xml 中设置参数；

|         Value                |        1         |      2        |      ...      |
|          :----               |       :----      |     :----     |      :----    |
|  点云处理 m_PcProcessNum      |                   |               |               |
|  点云检测 m_PcDetectNum       |   Second          |  Pointpillar  |               |
|  点云跟踪 m_PcTrackNum        |   隧道Sort         |               |               |
|  视频检测 m_VideoDetectNum    |   Yolov5          |    Yolov6     |               |
|  视频跟踪 m_VideoTrackNum     |   路口DeepSort     |               |               |
|  融合跟踪 m_FusionNum         |   Crossing        |    tunnel     |               |   
