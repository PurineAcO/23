import KU_v18 as KU
import math
global DefenseStage
global DefenseDeg
global DefenseMode
global DefenseJudge
global flagDefence
global flag2Defence
global DefenseAlt
flagDefence=[1,1,1,1]#防御flag
flag2Defence=[1,1,1,1]#防御flag
DefenseStage=[0,0,0,0]#每个飞机的防御阶段
DefenseDeg=[0,0,0,0]#每个飞机的防御航向
DefenseMode=[0,0,0,0]#0侧面导弹，1正面导弹,2导弹过于分散,3.两导弹来袭情况
DefenseJudge=[0,0,0,0]#0未判断，1已经判断
DefenseAlt=[0,0,0,0]#防御时飞机爬升高度

def DefenseAction(output_cmd,info,DroneID,plane_Yaw):
    """使用方法:
    import CommunicationTool
    def create_action_cmd(info, step_num):
        if (step_num <= jidong_time[0]):
            output_cmd =SendData()
            ......
            output_cmd=defensexample.DefenseAction2(output_cmd,info,info.DroneID,info.Yaw)
            ......
        return output_cmd 
        """
        # 防御躲避过程介绍：
        # 参数默认值：DefenseStage=[0,0,0,0]，DefenseDeg=[0,0,0,0]，DefenseMode=[0,0,0,0]，DefenseJudge=[0,0,0,0]
        #            flagDefence=[1,1,1,1] flag2Defence=[1,1,1,1]
        # 最终目的：通过机动动作增加导弹过载，使飞机在遭遇导弹来袭时能够躲避导弹
        # 1.判断导弹攻击类型阶段：
        #   首先判断导弹方向是否过于分散，如果分散且数量大于等于3则进入Mode=2，如果分散但仅有两枚导弹则进入Mode=3，
        #   若为正前方（相较于飞机速度方向夹角-45°-45°）则进入mode=1，其余方向则进入mode=0，
        #   其中DefenseJudge=0 表示未判断，1 表示已经判断，但考虑到探测导弹过程为逐渐探测，发现导弹未必探测到全部导弹，
        #   故增加DefenseJudge=2 用来确定探测到导弹探测完全（若仅有一个导弹时，DefenseJudge=1，自然便已经完全判断）
        # 2.姿态调整阶段：
        #   在执行躲避动作之前先通过机动飞行调整导弹与飞机的相对位置使躲避机动动作能够发挥出色效果
        #   若mode=0，则飞机向导弹速度方向转向至导弹速度方向与飞机航向相同,若mode=1，则飞机立即转180°（通过 flagDefence记录探测到导弹瞬间的飞机航向）
        #   若mode=2，则飞机仍按照原航向飞行，若mode=3，则飞机向两导弹角平分线方向转向至两导弹角平分线方向与飞机航向相同（通过 flag2Defence记录探测到两个导弹瞬间的导弹夹角方向）
        #   完成躲避预备动作后DefenseStage=1 并记录当前航向作为DefenseDeg，进入躲避机动阶段
        # 3.躲避机动阶段：
        #   躲避机动动作均以增加导弹过载为目的设计动作如下：
        #   若mode=0，则飞机沿DefenseDeg方向为轴做蛇形机动飞行,若mode=1，则飞机执行沿DefenseDeg方向最速爬升，
        #   若mode=2，则飞机执行沿DefenseDeg方向最速爬升，若mode=3，则飞机沿DefenseDeg方向为轴做蛇形机动飞行
        # 特别说明：如果在被导弹锁定情况下，有敌方飞机进入我方18000范围内，我方则直接进行爬升
        
        
   #信息获取
    JDDZ=KU.JDDZ(output_cmd,info,DroneID)
    RD=KU.RD()
    MissleDirectionList=KU.GetMissleDirection(info,DroneID,plane_Yaw)
    RelativeList=KU.GetRelative(info,DroneID)
    EnemyList=[]
    #获取敌机位置避免偷袭
    for j in range(len(info.FoundEnemyList)):
        if info.FoundEnemyList[j].TargetDis != 0:
            EnemyList.append(info.FoundEnemyList[j])
            
   #局势判断         
    # if  EnemyList and any(EnemyList[i].TargetDis<18000 and EnemyList[i].TargetAlt>info.Altitude+500 for i in range(len(EnemyList))):
    #     #被导弹锁定且有敌方飞机进入180000m，有偷袭风险
    #     DefenseMode[int((DroneID/100000)-1)]=2
    #     DefenseJudge[int((DroneID/100000)-1)]=2     
    if MissleDirectionList and 3*math.pi/2>=abs(max(RelativeList)-min(RelativeList))>=math.pi/2 and len(RelativeList)>=3 and not any(abs(RelativeList[i])<math.pi/4 for i in range(len(RelativeList))):
        #有三个以上较为分散的导弹
        # if DefenseMode[int((DroneID/100000)-1)]!=1 or ((DefenseMode[int((DroneID/100000)-1)]==1 and any(abs(RelativeList[i])<8*math.pi/15 for i in range(len(RelativeList))))):
        #     #之前未处理过正面导弹，或者分散导弹中存在前方导弹，则进入mode=2（准备爬升）
            DefenseMode[int((DroneID/100000)-1)]=2
            DefenseJudge[int((DroneID/100000)-1)]=2
        # elif DefenseMode[int((DroneID/100000)-1)]==1:
        #     #之前处理过正面导弹且不存在前方导弹，则继续转向180并准备蛇形机动
        #     DefenseMode[int((DroneID/100000)-1)]=1
        #     DefenseJudge[int((DroneID/100000)-1)]=1
    elif MissleDirectionList and 3*math.pi/2>=abs(max(RelativeList)-min(RelativeList))>=math.pi/2  and len(RelativeList)<3:
        #有两个较为分散的导弹
        DefenseMode[int((DroneID/100000)-1)]=3
        DefenseJudge[int((DroneID/100000)-1)]=2
    elif  any(abs(RelativeList[i])<math.pi/4 for i in range(len(RelativeList))) :
        #有正面导弹
        DefenseMode[int((DroneID/100000)-1)]=1
        DefenseJudge[int((DroneID/100000)-1)]=1
    elif DefenseJudge[int((DroneID/100000)-1)]==0 and abs(max(RelativeList)-min(RelativeList))<math.pi/2 :
        #侧面导弹，且集中于90度区域或仅有一个导弹
        DefenseMode[int((DroneID/100000)-1)]=0
        DefenseJudge[int((DroneID/100000)-1)]=1
    #防御阶段0，姿态调整阶段
    if DefenseStage[int((DroneID/100000)-1)]==0:
        #转向背离导弹方向
        if DefenseMode[int((DroneID/100000)-1)]==0:
            JDDZ.ZhuanWan(60,MissleDirectionList[0],8,2.0,1,600)
            if abs(RD.superr2d(info.Yaw)-MissleDirectionList[0])<10:               
                DefenseDeg[int((DroneID/100000)-1)]=RD.superr2d(info.Yaw)
                DefenseStage[int((DroneID/100000)-1)]=1 
                KU.flag2[int((DroneID/100000)-1)]=1
        elif DefenseMode[int((DroneID/100000)-1)]==1:
            #旋转近180度
            output_cmd.sPlaneControl.CmdID = 6
            output_cmd.sPlaneControl.VelType = 0
            output_cmd.sPlaneControl.CmdSpd = 2.0
            output_cmd.sPlaneControl.CmdPhi = 60
            output_cmd.sPlaneControl.CmdNy = 8
            output_cmd.sPlaneControl.isApplyNow = True
            output_cmd.sPlaneControl.CmdThrust = 600
            output_cmd.sPlaneControl.ThrustLimit = 600
            if flagDefence[int((DroneID/100000)-1)]==1:#探测正面导弹来向
                if RelativeList[0]>=0:
                    DefenseDeg[int((DroneID/100000)-1)]=(RD.superr2d(info.Yaw)+150)%360
                    output_cmd.sPlaneControl.TurnDirection = -1#向左后转
                elif RelativeList[0]<0:
                    DefenseDeg[int((DroneID/100000)-1)]=(RD.superr2d(info.Yaw)-150)%360
                    output_cmd.sPlaneControl.TurnDirection = 1#向右后转
                flagDefence[int((DroneID/100000)-1)]=0
            output_cmd.sPlaneControl.CmdHeadingDeg = DefenseDeg[int((DroneID/100000)-1)]
            if abs(RD.superr2d(info.Yaw)-DefenseDeg[int((DroneID/100000)-1)])<10:
                DefenseAlt[int((DroneID/100000)-1)]=info.Altitude+5000 if info.Altitude<10000 else info.Altitude+4000
                DefenseStage[int((DroneID/100000)-1)]=1
        elif DefenseMode[int((DroneID/100000)-1)]==2:
            #保持原航向
             DefenseAlt[int((DroneID/100000)-1)]=info.Altitude+5000 if info.Altitude<10000 else info.Altitude+4000
             DefenseStage[int((DroneID/100000)-1)]=1
             DefenseStage[int((DroneID/100000)-1)]=1
             DefenseDeg[int((DroneID/100000)-1)]=RD.superr2d(info.Yaw)
        elif DefenseMode[int((DroneID/100000)-1)]==3:
            #转向两导弹角平分线方向
            if flag2Defence[int((DroneID/100000)-1)]==1: 
                DefenseDeg[int((DroneID/100000)-1)]=(MissleDirectionList[0]+MissleDirectionList[1])/2
                if abs(MissleDirectionList[0]-MissleDirectionList[1])>180:
                    DefenseDeg[int((DroneID/100000)-1)]=((MissleDirectionList[0]+MissleDirectionList[1])/2)+180
                flag2Defence[int((DroneID/100000)-1)]=0
            JDDZ.ZhuanWan(60,DefenseDeg[int((DroneID/100000)-1)],8,2.0,1,600)
            if abs(RD.superr2d(info.Yaw)-DefenseDeg[int((DroneID/100000)-1)])<10: 
                DefenseDeg[int((DroneID/100000)-1)]=RD.superr2d(info.Yaw)
                DefenseStage[int((DroneID/100000)-1)]=1 
                KU.flag2[int((DroneID/100000)-1)]=1
    #防御阶段1，执行躲避机动动作            
    elif DefenseStage[int((DroneID/100000)-1)]==1:
        if DefenseMode[int((DroneID/100000)-1)]==0:#蛇形机动躲避
            JDDZ.SheXing(45,DefenseDeg[int((DroneID/100000)-1)]+40,DefenseDeg[int((DroneID/100000)-1)]-40,4,2.0,160)
        elif DefenseMode[int((DroneID/100000)-1)]==1:#最速爬升躲避
            JDDZ.PaSheng(DefenseDeg[int((DroneID/100000)-1)],3.0, DefenseAlt[int((DroneID/100000)-1)],500,200)
        elif DefenseMode[int((DroneID/100000)-1)]==2:#最速爬升躲避
            JDDZ.PaSheng(DefenseDeg[int((DroneID/100000)-1)],3.0, DefenseAlt[int((DroneID/100000)-1)],500,200)
        elif DefenseMode[int((DroneID/100000)-1)]==3:#蛇形机动躲避
            JDDZ.SheXing(45,DefenseDeg[int((DroneID/100000)-1)]+40,DefenseDeg[int((DroneID/100000)-1)]-40,4,2.0,160)