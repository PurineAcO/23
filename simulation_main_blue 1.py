from CommunicationTool import *
import sys
jidong_time=[500000]

def create_action_cmd(info, step_num):
    global launchFlag
    if (step_num <= jidong_time[0]):
        output_cmd = SendData()
        output_cmd.sPlaneControl.CmdIndex = 1
        output_cmd.sPlaneControl.CmdID = 2
        if (step_num == jidong_time[0]):
            output_cmd.sPlaneControl.isApplyNow = False
        output_cmd.sPlaneControl.isApplyNow = True
        output_cmd.sPlaneControl.CmdHeadingDeg = 180
        output_cmd.sPlaneControl.CmdSpd = 2
        output_cmd.sPlaneControl.VelType = 0
        output_cmd.sPlaneControl.ThrustLimit = 100
        if(info.DroneID==800000):
            if info.AttackEnemyList[1].EnemyID == 200000:
                with open('output1.txt','a',encoding='utf-8') as f:
                    sys.stdout = f
                    print(vars(info.AttackEnemyList[0]),"\n")
                    print(vars(info.AttackEnemyList[1]),"\n")
                    print(vars(info.AttackEnemyList[2]),"\n")
                    print(vars(info.AttackEnemyList[3]),"\n")
                    print("___________________________________________\n")
                if info.AttackEnemyList[1].NTSstate == 1:
                    output_cmd.sSOCtrl.isNTSAssigned = 1
                    output_cmd.sSOCtrl.NTSEntityIdAssigned = info.AttackEnemyList[1].EnemyID
                    output_cmd.sOtherControl.isLaunch = 0
                elif info.AttackEnemyList[0].NTSstate == 2 and launchFlag == 1:
                    output_cmd.sOtherControl.isLaunch = 1
                    launchFlag = 0

            if info.AttackEnemyList[1].EnemyID == 100000:
                with open('output1.txt','a',encoding='utf-8') as f:
                    sys.stdout = f
                    print(vars(info.AttackEnemyList[0]),"\n")
                    print(vars(info.AttackEnemyList[1]),"\n")
                    print(vars(info.AttackEnemyList[2]),"\n")
                    print(vars(info.AttackEnemyList[3]),"\n")
                    print("___________________________________________\n")
                if info.AttackEnemyList[1].NTSstate == 1:
                    output_cmd.sSOCtrl.isNTSAssigned = 1
                    output_cmd.sSOCtrl.NTSEntityIdAssigned = info.AttackEnemyList[1].EnemyID
                    output_cmd.sOtherControl.isLaunch = 0
                elif info.AttackEnemyList[0].NTSstate == 2 and launchFlag == 1:
                    output_cmd.sOtherControl.isLaunch = 1
                    launchFlag = 0