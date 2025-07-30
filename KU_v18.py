from CommunicationTool import *
import math
import numpy as np


global flag
global flag2
global TurnDirection
flag=1#一定要有 并且 在主函数create_action_cmd中添加global flag
flag2=[1,1,1,1]
TurnDirection=0
class RD:
    """包含4个方法分别为`r2d`,`d2r`,`superr2d`,`superr2d`,用于将弧度制和角度制相互转换"""
    def __init__(self):
        self.rfactor=57.32484
    def r2d(self,r):
        return r*self.rfactor
    def d2r(self,d):
        return d/self.rfactor
    def superd2r(self,d):
        d=d%360
        return self.d2r(d) if d<=180 else self.d2r(d-360)
    def superr2d(self,r):
        return self.r2d(r) if r>=0 else self.r2d(r)+360
    def LongituteDis(self,LonDiffer,Lat):
        """计算经度差对应的距离,单位为米"""
        return LonDiffer*(math.pi/180)*6378137.0*math.cos(self.d2r(Lat))
    def LatitudeDis(self,LatDiffer):
        """计算纬度差对应的距离,单位为米"""
        return LatDiffer*(math.pi/180)*6378137.0
    def GetPosition(self,info,DroneID):
        """获取DroneID对应无人机的位置信息，返回值为角度制的经纬度和高度"""
        if info.DroneID==DroneID:
            position=(self.r2d(info.Longtitude),self.r2d(info.Latitude),info.Altitude)
        return position
    
RDer=RD()
      

class JDDZ:
    """机动动作类,需要提前依次传入`output_cmd`和`info`"""
    def __init__(self,output_cmd,info,DroneID):
        self.output_cmd=output_cmd
        self.info=info
        self.DroneID=DroneID

    def PingFei(self,Deg,Spd,Thrust=100): 
        """平飞,传入参数:平飞方向`Deg`,预设速度`Spd`,油门(如果不传入默认`100`)"""
        self.output_cmd.sPlaneControl.CmdID = 1
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg%360
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust
    
    def JiaJianSu(self,Deg,Spd,Thrust=120):
        """加减速,传入参数:方向`Deg`,预设速度`Spd`,油门(如果不传入默认`120`)"""
        self.output_cmd.sPlaneControl.CmdID = 2
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg%360
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust 
    
    def PaSheng(self,Deg,Spd,Alt,Thrust1=120,Thrust2=100):
        """最速爬升到`Alt`高度后沿着`Deg`方向平飞,`Thrust1`为最速爬升时的推力(默认`120`),`Thrust2`为平飞时的推力(默认`100`)"""
        self.output_cmd.sPlaneControl.CmdID = 3
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg%360
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.CmdThrust = Thrust1
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust1
        if self.info.Altitude>=Alt:
            self.output_cmd=self.PingFei(Deg,Spd,Thrust2)
    
    def FuChong(self,Spd,Alt,PitchDeg,Deg,Thrust):
        """俯冲,传入参数:预设速度`Spd`,俯冲目标海拔`Alt`,航迹倾角`PitchDeg`(**-90~+90**),俯冲方向`Deg`,油门"""
        self.output_cmd.sPlaneControl.CmdID = 7
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg%360
        self.output_cmd.sPlaneControl.CmdPitchDeg = PitchDeg
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust 
        self.output_cmd.sPlaneControl.CmdAlt=Alt
    def ZhuanWan(self,Phi,Deg,Ny,Spd,TurnMode,Thrust=120):
        """转弯,传入参数:滚转角`Phi`,转弯方向`Deg`,过载`Ny`,预设速度`Spd`,转弯模式(`1`为锐角转弯,`2`为钝角转弯),油门(默认`120`)"""
        global flag2
        global TurnDirection
        Deg=Deg%360
        self.output_cmd.sPlaneControl.CmdID = 6
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg
        self.output_cmd.sPlaneControl.CmdPhi = Phi
        self.output_cmd.sPlaneControl.CmdNy = Ny
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.CmdThrust = Thrust
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust
        judge_deg=0 #是否大于180，大于为1
        judge_mode=0 #目标方向与当前方向角度大小关系，大于为1，小于为0
        if(abs(Deg-RDer.superr2d( self.info.Yaw)))>=180:
            judge_deg=1
        else:
            judge_deg=0
        if(Deg-RDer.superr2d( self.info.Yaw))>0:
            judge_mode=1
        else:
            judge_mode=0    
        if judge_mode==1 and judge_deg==1 and flag2[int((self.DroneID/100000)-1)]==1:
            flag2[int((self.DroneID/100000)-1)]=0
            if TurnMode==1:
                TurnDirection = -1    
            elif TurnMode==2:
                TurnDirection = 1
        elif judge_mode==1 and judge_deg==0 and flag2[int((self.DroneID/100000)-1)]==1:
             if TurnMode==1:
                TurnDirection = 1
                flag2[int((self.DroneID/100000)-1)]=0
             elif TurnMode==2:
                TurnDirection = -1
                flag2[int((self.DroneID/100000)-1)]=0
        elif judge_mode==0 and judge_deg==1 and flag2[int((self.DroneID/100000)-1)]==1:
            if TurnMode==1:
                TurnDirection = 1
                flag2[int((self.DroneID/100000)-1)]=0
            elif TurnMode==2:
                TurnDirection = -1
                flag2[int((self.DroneID/100000)-1)]=0
        elif judge_mode==0 and judge_deg==0 and flag2[int((self.DroneID/100000)-1)]==1:
            if TurnMode==1:
                TurnDirection = -1
                flag2[int((self.DroneID/100000)-1)]=0
            elif TurnMode==2:
                TurnDirection = 1
                flag2[int((self.DroneID/100000)-1)]=0
        self.output_cmd.sPlaneControl.TurnDirection = TurnDirection


    def SheXing(self,Phi,Deg1,Deg2,Ny,Spd,Thrust=120):
        """蛇形,传入参数:滚转角`Phi`,起始航向`Deg1`,终止航向`Deg2`,过载`Ny`,预设速度`Spd`,油门(默认`120`),注意:需要确保航向夹在两个角度之间"""
        global flag
        self.output_cmd.sPlaneControl.CmdID = 6
        self.output_cmd.sPlaneControl.VelType = 0
        self.output_cmd.sPlaneControl.CmdPhi = Phi
        self.output_cmd.sPlaneControl.CmdSpd = Spd
        self.output_cmd.sPlaneControl.CmdNy = Ny
        self.output_cmd.sPlaneControl.isApplyNow = True
        self.output_cmd.sPlaneControl.CmdThrust = Thrust
        self.output_cmd.sPlaneControl.ThrustLimit = Thrust
        judge_deg=0#0为夹角小于180，1为夹角大于180 
        nega=0#判断两角度的弧度制是否为同号，nega%2==0为同号 ==1为异号
        Deg1=Deg1%360
        Deg2=Deg2%360
        if Deg1>Deg2:#使Deg1为较小的度数
            temp=Deg1
            Deg1=Deg2
            Deg2=temp
        if Deg2-Deg1>180:
            judge_deg=1
        Yaw1_my=0
        Yaw2_my=0
        if Deg1<=180 :
            Yaw1_my=Deg1*math.pi/180
        else:
            Yaw1_my=(Deg1-360)*math.pi/180
            nega=nega+1
        if Deg2<=180 :
            Yaw2_my=Deg2*math.pi/80
        else:
            Yaw2_my=(Deg2-360)*math.pi/180
            nega=nega+1
        if judge_deg==0:    
            if flag == 1 :
                self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg1
                self.output_cmd.sPlaneControl.TurnDirection = -1
                if nega%2==0:
                    if  self.info.Yaw <=Yaw1_my:
                        flag = 0
                if nega%2==1:
                    if self.info.Yaw<=Yaw1_my and self.info.Yaw>0:
                        flag = 0
            elif flag==0:
                self.output_cmd.sPlaneControl.TurnDirection = 1
                self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg2
                if nega%2==0:
                    if self.info.Yaw >= Yaw2_my/180:
                        flag=1
                if nega%2==1:
                    if self.info.Yaw >= Yaw2_my/180 and self.info.Yaw<0:
                        flag=1
        elif judge_deg==1:
            if flag == 1 :
                self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg1
                self.output_cmd.sPlaneControl.TurnDirection = 1
                if self.info.Yaw>=Yaw1_my:
                    flag=0
            elif flag==0:
                self.output_cmd.sPlaneControl.CmdHeadingDeg = Deg2
                self.output_cmd.sPlaneControl.TurnDirection = -1
                if self.info.Yaw<=Yaw2_my:
                    flag=1

    
def CABP(x0, y0, z0, vn1, ve1, l, h1, theta_rad):
    """`A`的经度、纬度、海拔、北向速度、东向速度、B的直线距离、海拔、方向角 ————> `B`的经度、纬度、海拔"""
    global EARTH_R,rfactor
    EARTH_R = 6378137.0
    rfactor=57.32484
    A_rad = math.atan2(ve1, vn1)
    nt_rad = A_rad + theta_rad
    delta_h = abs(h1 - z0)
    pingmiandis = math.sqrt(abs(l**2 - delta_h**2))  
    A_weidu = y0
    A_jingdu = x0 
    raddistance = pingmiandis / EARTH_R
    B_weidu = math.asin(
        math.sin(A_weidu) * math.cos(raddistance) +
        math.cos(A_weidu) * math.sin(raddistance) * math.cos(nt_rad)
    )
    B_jingdu = A_jingdu + math.atan2(
        math.sin(nt_rad) * math.sin(raddistance) * math.cos(B_weidu),
        math.cos(raddistance) - math.sin(A_weidu) * math.sin(B_weidu)
    )
    x1 = RDer.r2d(B_jingdu)
    y1 = RDer.r2d(B_weidu)
    # 处理经度溢出
    x1 = (x1 + 180) % 360 - 180
    return RDer.d2r(x1), RDer.d2r(y1), h1

def jinggao(output_cmd,info, step_num,jidong_time):
    """打印所有被导弹锁定的飞机编号和导弹的方位角"""
    warnid=[0,0,0,0]

    if(step_num<=jidong_time[0]):
        #FIXME:这段机动动作还有必要吗,应用在什么场景?
        output_cmd.sPlaneControl.CmdIndex = 1
        output_cmd.sPlaneControl.CmdID = 2
        if (step_num == jidong_time[0]):
            output_cmd.sPlaneControl.isApplyNow = False
        output_cmd.sPlaneControl.isApplyNow = True
        output_cmd.sPlaneControl.CmdHeadingDeg = 180
        output_cmd.sPlaneControl.CmdSpd = 2
        output_cmd.sPlaneControl.VelType = 0
        output_cmd.sPlaneControl.ThrustLimit = 120
        for k in range(1,5):#将被锁定的飞机列入警告表
            if(info.DroneID==k*100000 and info.isMisWarning == True):
                warnid[k-1]=info.DroneID
        for i in range(0,4):#遍历警告表
            if(warnid[i]!=0):
                for j in range(0,10):#打印每架飞机
                 if(info.AlarmList[j].MisAzi!=0 and info.AlarmList[j].AlarmType=="导弹" ):#打印被导弹锁定的飞机编号和导弹的方位角
                    print(info.AlarmList[j].AlarmType,info.AlarmList[j].MisAzi,info.AlarmList[j].AlarmID,info.DroneID)          

    return output_cmd  

def GetRelative(info,DroneID):
    """获取锁定`DroneID`对应的飞机的导弹相对于该飞机的相对角度,返回值为弧度制列表"""
    if info.DroneID==DroneID:
        RelativeList=[]
        for index in range(len(info.AlarmList)):
            if info.AlarmList[index].AlarmType=="导弹" and not info.AlarmList[index].MisAzi == 0:
                RelativeList.append(info.AlarmList[index].MisAzi)
        return RelativeList
    else:
        return None
    
def GetMissleDirection(info,DroneID,plane_Yaw):
    """获取锁定`DroneID`对应的飞机的导弹相对于该飞机的相对角度,返回值为角度制列表"""
    if info.DroneID==DroneID:
         RelativeList=GetRelative(info,DroneID)
         DirectionList=[]
         for index in range(len(RelativeList)):
             DirectionList.append((180+(RDer.superr2d(plane_Yaw)+RDer.superr2d(RelativeList[index])))%360)
         return DirectionList
    else:
         return None

"""攻击状态全局变量区,注意关键词"""
attackmap=[0,{500000:True,600000:True,700000:True,800000:True},
            {500000:True,600000:True,700000:True,800000:True},
            {500000:True,600000:True,700000:True,800000:True},
            {500000:True,600000:True,700000:True,800000:True}]
missilecnt=[0,0,0,0,0]
actioncnt=0
attackstate=404

class attackmethod(JDDZ):
    """首先需要传入`output_cmd`和`info`,尽管VS Code并没有提示"""
    def __init__(self,output_cmd,info):
        super().__init__(output_cmd,info)
        self.lenattack=0
        for target in self.info.AttackEnemyList:
            if target.EnemyID != 0:
                self.lenattack+=1

    def fadan(self):
        self.output_cmd.sOtherControl.isLaunch=1
        # with open('output.txt', 'a', encoding='utf-8') as f:
        #     sys.stdout = f
        #     print("已经锁定,对上面锁定的发射")
    
    def suoding(self,EnemyID):
        self.output_cmd.sOtherControl.isLaunch=0
        self.output_cmd.sSOCtrl.isNTSAssigned=1
        self.output_cmd.sSOCtrl.NTSEntityIdAssigned=EnemyID
        # with open('output.txt', 'a', encoding='utf-8') as f:
        #     sys.stdout = f
        #     print("锁定",EnemyID)

        
    def attack1(self,DroneID,missilenum):
        """需要指定发弹飞机`DroneID`和发弹数量`missilenum`"""
        global actioncnt,attackmap,missilecnt
        if self.info.DroneID == DroneID and self.lenattack!=0 and missilecnt[DroneID//100000]<min(self.lenattack,missilenum) :
            if self.info.AttackEnemyList[actioncnt].TargetDis<=25000 and attackmap[DroneID//100000][self.info.AttackEnemyList[actioncnt].EnemyID]==True:
                if self.info.AttackEnemyList[actioncnt].NTSstate == 2:
                    self.fadan()
                    attackmap[DroneID//100000][self.info.AttackEnemyList[actioncnt].EnemyID]=False
                    actioncnt=(actioncnt+1)%self.lenattack
                    missilecnt[DroneID//100000]+=1
                else:
                    self.suoding(self.info.AttackEnemyList[actioncnt].EnemyID)
            else:actioncnt=(actioncnt+1)%self.lenattack

            # with open('output.txt', 'a', encoding='utf-8') as f:
            #     sys.stdout = f
            #     print("actioncnt:",actioncnt,"missilecnt:",missilecnt)

        
    def attacktest(self,DroneID,EnemyID):
        """对2架飞机反复发弹,要求该2架飞机在某个范围内,不可改地定义为`40000`"""
        #BUG:只对500000发弹
        global attackstate
        if self.info.DroneID==DroneID and len(self.info.AttackEnemyList)!=0:
            if attackstate==1:
                for i in range(1,len(self.info.AttackEnemyList)):
                    if 0<self.info.AttackEnemyList[i].TargetDis<=40000:
                        self.suoding(self.info.AttackEnemyList[i].EnemyID)
                        attackstate=403
                    break
            elif attackstate==403:
                self.fadan()
                attackstate=404
            else:
                for target in self.info.AttackEnemyList:
                    if target.EnemyID==EnemyID and attackstate==404:
                        if target.NTSstate==2:
                            self.fadan()
                            attackstate=1
                        else:
                            self.suoding(EnemyID)
                            attackstate=2
                    elif target.EnemyID==EnemyID and attackstate==2:
                        if target.NTSstate==2:
                            self.fadan()
                            attackstate=1
                        else:
                            self.suoding(EnemyID)
                            attackstate=2
    

def GetTargetID(info,DroneID):
    """获取Drone的FoundEnemyList中距离最近的敌方飞机的ID，未探测到目标则返回404"""
    if info.DroneID==DroneID:
        Enemy=[]
        for i in range(len(info.FoundEnemyList)):
            if info.FoundEnemyList[i].TargetDis!=0:
                Enemy.append(info.FoundEnemyList[i])
        if len(Enemy)!=0:
            for j in range(len(Enemy)):
                if Enemy[j].TargetDis==min([Enemy[i].TargetDis for i in range(len(Enemy))]):
                    return Enemy[j].EnemyID
        else:
            return 404
            
class Mp(JDDZ):#建立战场类
    def __init__(self,lon_start,lon_end,lat_start,lat_end,ChiliParameter):# 初始化函数，用于创建一个战场区域对象
        self.lon_start = lon_start # 设置战场区域的起始经度
        self.lon_end = lon_end# 设置战场区域的结束经度
        self.lat_start = lat_start# 设置战场区域的起始纬度
        self.lat_end = lat_end# 设置战场区域的结束纬度
        self.ChiliParameter=ChiliParameter # 设置斥力参数

    def is_inside(self,position):
        """Position为GetPosition（info，DroneID）的返回值，判断飞机是否在战场内"""
        Plane_lon,Plane_lat,Plane_alt=position
        if self.lon_start <= Plane_lon <= self.lon_end and self.lat_start <= Plane_lat <= self.lat_end:
            return True
        else:
            return False
        
    def distance2boundary(self,position):
        """返回值为东向力距离-西向力距离，北向力距离-南向力距离"""
        EARTH_R = 6378137.0
        Plane_lon,Plane_lat,Plane_alt=position
        DisMapLonLeft=(Plane_lon-self.lon_start)*(math.pi/180)*EARTH_R*math.cos(RDer.d2r(Plane_lat))#求解与各边界的距离
        DisMapLonRight=(Plane_lon-self.lon_end)*(math.pi/180)*EARTH_R*math.cos(RDer.d2r(Plane_lat))
        DisMapLatUp=(Plane_lat-self.lat_end)*(math.pi/180)*EARTH_R
        DisMapLatDown=(Plane_lat-self.lat_start)*(math.pi/180)*EARTH_R
        Fright=self.ChiliParameter/abs(DisMapLonLeft)#正数，东向力
        Fleft=-self.ChiliParameter/abs(DisMapLonRight)#负数，西向力
        Fup=self.ChiliParameter/abs(DisMapLatDown)#正数，北向力
        Fdown=-self.ChiliParameter/abs(DisMapLatUp)#负数，南向力
        return Fright+Fleft,Fup+Fdown,0
    
class Obstacle:#建立威胁区类
    def __init__(self,info,lon,lat,alt,radius,ChiliParameter):
        self.info = info
        self.lon = lon
        self.lat = lat
        self.alt = alt
        self.radius = radius
        self.ChiliParameter=ChiliParameter
    def is_inside(self,DroneID):
        """
        DroneID:我方飞机ID
        判断我方飞机是否在威胁区中
        """
        if self.info.DroneID==DroneID:
            x=RDer.superr2d(self.info.Longtitude)
            y=RDer.superr2d(self.info.Latitude)
            z=self.info.Altitude
            if (RDer.LongituteDis(self.lon-x,y))**2+(RDer.LatitudeDis(y-self.lat))**2+(z-self.alt)**2<=self.radius**2:
                return True
            else:
                return False
    def distance2Obs(self,position):
        """依次返回东、北、上三个方向的斥力信息"""
        Plane_lon,Plane_lat,Plane_alt=position
        Length=math.sqrt(RDer.LongituteDis(self.lon-Plane_lon,self.lat)**2+RDer.LatitudeDis(self.lat-Plane_lat)**2)
        theta=math.acos((RDer.LongituteDis((Plane_lon-self.lon),self.lat))/Length)
        cos = abs(math.cos(theta))
        sin = abs(math.sin(theta))
        if Plane_lon>=self.lon and RDer.LongituteDis((Plane_lon-self.lon),self.lat)-(math.sqrt(self.radius**2-Plane_alt**2))*cos>0:
            DisEast=RDer.LongituteDis((Plane_lon-self.lon),self.lat)-(math.sqrt(self.radius**2-Plane_alt**2))*cos#飞机在障碍的东面为正数，在西面为负数
        elif Plane_lon>=self.lon and RDer.LongituteDis((Plane_lon-self.lon),self.lat)-(math.sqrt(self.radius**2-Plane_alt**2))*cos<0:
            DisEast=-(RDer.LongituteDis((Plane_lon-self.lon),self.lat)-(math.sqrt(self.radius**2-Plane_alt**2))*cos)*100
        elif Plane_lon<self.lon and RDer.LongituteDis((self.lon-Plane_lon),self.lat)-(math.sqrt(self.radius**2-Plane_alt**2))*cos>0:
            DisEast=-(RDer.LongituteDis((self.lon-Plane_lon),self.lat)-(math.sqrt(self.radius**2-Plane_alt**2))*cos)
        elif Plane_lon<self.lon and RDer.LongituteDis((self.lon-Plane_lon),self.lat)-(math.sqrt(self.radius**2-Plane_alt**2))*cos<0:
            DisEast=(RDer.LongituteDis((self.lon-Plane_lon),self.lat)-(math.sqrt(self.radius**2-Plane_alt**2))*cos)*100
        if Plane_lat>=self.lat and RDer.LatitudeDis(Plane_lat-self.lat)-(math.sqrt(self.radius**2-Plane_alt**2))*sin>0:
            DisNorth=RDer.LatitudeDis(Plane_lat-self.lat)-(math.sqrt(self.radius**2-Plane_alt**2))*sin#飞机在障碍的北面为正数，在南面为负数
        elif Plane_lat>=self.lat and RDer.LatitudeDis(Plane_lat-self.lat)-(math.sqrt(self.radius**2-Plane_alt**2))*sin<0:
            DisNorth=-(RDer.LatitudeDis(Plane_lat-self.lat)-(math.sqrt(self.radius**2-Plane_alt**2))*sin)*100
        elif Plane_lat<self.lat and RDer.LatitudeDis(self.lat-Plane_lat)-(math.sqrt(self.radius**2-Plane_alt**2))*sin>0:
            DisNorth=RDer.LatitudeDis(Plane_lat-self.lat)+(math.sqrt(self.radius**2-Plane_alt**2))*sin
        elif Plane_lat<self.lat and RDer.LatitudeDis(self.lat-Plane_lat)-(math.sqrt(self.radius**2-Plane_alt**2))*sin<0:
            DisNorth=-(RDer.LatitudeDis(Plane_lat-self.lat)+(math.sqrt(self.radius**2-Plane_alt**2))*sin)*100
        return self.ChiliParameter/DisEast,self.ChiliParameter/DisNorth,0
    def LeftDistance2Obs(self,position):
        """同水平面上我放飞机与威胁区所围圆形的剩余距离"""
        Plane_lon,Plane_lat,Plane_alt=position
        Length=math.sqrt(RDer.LongituteDis(self.lon-Plane_lon,self.lat)**2+RDer.LatitudeDis(self.lat-Plane_lat)**2)-(math.sqrt(self.radius**2-Plane_alt**2))
        return Length
def TargetDist(DroneID,TargetID,info,YinliParameter):
    """依次返回东、北、上三个方向的斥力信息"""
    Plane_lon,Plane_lat,Plane_alt=RDer.GetPosition(info,DroneID)
    if info.DroneID==DroneID:
        x=info.Longtitude
        y=info.Latitude
        z=info.Altitude
        vn1=info.V_N
        ve1=info.V_E
        index=-1
        #判断是否找到目标敌机，若没有则引力返回0，0，0
        for i in range(len(info.FoundEnemyList)):
            if info.FoundEnemyList[i].EnemyID==TargetID and info.FoundEnemyList[i].TargetDis != 0:
                index=i
                break
        if index==-1:
            return 0,0,0
        #若找到目标敌机，则引力返回目标敌机与自身之间的距离和引力系数的乘积
        else:
            l=info.FoundEnemyList[i].TargetDis
            h1=info.FoundEnemyList[i].TargetAlt
            theta=info.FoundEnemyList[i].TargetYaw
            goal_lon,goal_lat,goal_alt=CABP(x,y,z,vn1,ve1,l,h1,theta)
            goal_lon=RDer.superr2d(goal_lon)
            goal_lat=RDer.superr2d(goal_lat)
            goal_alt=goal_alt
        DisEast=RDer.LongituteDis((goal_lon-Plane_lon),Plane_lat)#飞机在目标东面为正数，在西面为负数
        DisNorth=RDer.LatitudeDis(goal_lat-Plane_lat)#飞机在目标北面为正数，在南面为负数
        DisUp=(goal_alt-Plane_alt)
        DisEast=90000 if DisEast>90000 else DisEast
        DisEast=-90000 if DisEast<-90000 else DisEast
        DisNorth=90000 if DisNorth>90000 else DisNorth
        DisNorth=-90000 if DisNorth<-90000 else DisNorth
        return YinliParameter*DisEast,YinliParameter*DisNorth,YinliParameter*DisUp
     

global ZhuiJiMode     
ZhuiJiMode=[0,0,0,0]#0表示探测到目标，1表示没有目标
def APF_Valpha(output_cmd,info,DroneID,TargetID,mp,obstacle,Spd_PingFei,Thrust_PingFei,Spd_PaSheng,Thrust_PaSheng,YinliParameter):
    """TargetID"""
    Foundflag=0
    if TargetID==0:
        TargetID=GetTargetID(info,DroneID)
    if TargetID==404:
        TargetID=GetTargetID(info,DroneID)
    if info.DroneID==DroneID:
        JDDZer=JDDZ(output_cmd,info,DroneID)
        Mapper=Mp(mp.lon_start,mp.lon_end,mp.lat_start,mp.lat_end,mp.ChiliParameter)
        ob=Obstacle(info,obstacle.lon,obstacle.lat,obstacle.alt,obstacle.radius,obstacle.ChiliParameter)
        ForceEast1,ForceNorth1,ForceUp1=TargetDist(DroneID,TargetID,info,YinliParameter)#获取引力信息
        ForceEast2,ForceNorth2,ForceUp2=Mapper.distance2boundary(RDer.GetPosition(info,DroneID))#获取战场边界斥力信息
        ForceEast3,ForceNorth3,ForceUp3=ob.distance2Obs(RDer.GetPosition(info,DroneID))#获取危险区斥力信息
       #判断平飞速度是否能够追上敌机
        for i in range(len(info.FoundEnemyList)): 
            if info.FoundEnemyList[i].EnemyID==TargetID and info.FoundEnemyList[i].TargetDis != 0:
                Foundflag=1
                if (info.FoundEnemyList[i].TargetV_N)*(info.V_N)>0 or (info.FoundEnemyList[i].TargetV_E)*(info.V_E)>0: #判断敌方在接近还是远离我方
                    if info.Mach_M <info.FoundEnemyList[i].TargetMach_M:#判断我方速度是否小于敌方速度
                       Spd_PingFei=info.FoundEnemyList[i].TargetMach_M+0.3#如果小于敌方速度，则将平飞速度设为敌方速度+0.3
                       Thrust_PingFei=(Spd_PingFei)*100+40#调节油门大小
        if Foundflag==0:
            TargetID=GetTargetID(info,DroneID) 
            
        #引力为0代表没有探测到目标，将原地盘旋等待雷达探测到目标
        if ForceEast1==0 and ForceNorth1==0 and ForceUp1==0 and math.sqrt((ForceEast2+ForceEast3)**2+(ForceNorth2+ForceNorth3)**2)<1000:
            ZhuiJiMode[int(DroneID/100000)-1]=0
        #引力不为0代表探测到目标，此时需要判断是否需要考虑斥力
        elif ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))>6000:#离危险区较远可以忽略危险区斥力，直接追击敌方
            ZhuiJiMode[int(DroneID/100000)-1]=1
        elif ob.LeftDistance2Obs(RDer.GetPosition(info,DroneID))<6000:#离危险区较近需要考虑斥力影响
            ZhuiJiMode[int(DroneID/100000)-1]=2
        #与未找到敌机盘旋过程相配合，避免在盘旋过程中进入威胁区而爆炸    
        elif  ZhuiJiMode[int(DroneID/100000)-1]==0 and math.sqrt((ForceEast2+ForceEast3)**2+(ForceNorth2+ForceNorth3)**2)>3000:
            ZhuiJiMode[int(DroneID/100000)-1]=2
            
        if ZhuiJiMode[int(DroneID/100000)-1]==0:#盘旋等待敌方出现
            JDDZer.ZhuanWan(60,RDer.superr2d(info.Yaw)+30,4,Spd_PingFei,1,Thrust=Thrust_PingFei)
        #离威胁区较远可以忽略威胁区斥力，直接追击敌方
        elif ZhuiJiMode[int(DroneID/100000)-1]==1:
            theta_rad=np.arctan2(ForceEast1+ForceEast2,ForceNorth1+ForceNorth2)
            theta_deg=RDer.superr2d(theta_rad)
            if abs(RDer.superr2d(info.Yaw)-theta_deg)>20 or ob.is_inside(DroneID):
                JDDZer.ZhuanWan(60,theta_deg,6,Spd_PingFei,Spd_PingFei,Thrust=Thrust_PingFei)
            elif abs(RDer.superr2d(info.Yaw)-theta_deg)<=20:
                JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei)
        #离威胁区较近需要考虑斥力影响        
        elif ZhuiJiMode[int(DroneID/100000)-1]==2:
            matrix=np.array([[ForceEast1,ForceNorth1,ForceUp1],[ForceEast2,ForceNorth2,ForceUp2],[ForceEast3,ForceNorth3,ForceUp3]])#利用一个矩阵来存储三组力
            Force_sums = np.sum(matrix, axis=0)#对矩阵按列求和，得到东、北、上三个方向的力的分量大小（为负数即为反向）
            theta_rad=np.arctan2(Force_sums[0],Force_sums[1])#根据力的分量大小求出水平面上力的方向
            theta_deg=RDer.superr2d(theta_rad)#转换为平飞航向代码所需的航向（角度制）
            if abs(RDer.superr2d(info.Yaw)-theta_deg)>20:
                JDDZer.ZhuanWan(60,theta_deg,6,Spd_PingFei,1,Thrust=Thrust_PingFei)
            elif abs(RDer.superr2d(info.Yaw)-theta_deg)<=20:
                JDDZer.PingFei(theta_deg,Spd_PingFei,Thrust=Thrust_PingFei)
        #当敌方飞机高于我方400m以上时，我方将爬升来追击敌方        
        if  ForceUp1>400*YinliParameter:
            JDDZer.PaSheng(theta_deg,Spd_PaSheng,info.Altitude+200,Thrust_PaSheng,Thrust_PingFei)
        #当敌方低于我方1000m以上时，我方向下俯冲500m来追击敌方 
        if ForceUp1<-1000*YinliParameter:
            JDDZer.FuChong(Spd_PaSheng,info.Altitude-500,-80,theta_deg,Thrust_PaSheng)           