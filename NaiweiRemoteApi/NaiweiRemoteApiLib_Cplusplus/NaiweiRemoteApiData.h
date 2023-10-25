#pragma once

namespace NaiweiRobot
{
    //uint16_t AxesCount = 9;//总轴数

    /// <summary>
    /// 操作模式枚举
    /// </summary>
    enum OpModeType : int16_t
    {
        Manual = 0,//手动模式
        Auto = 1,//自动模式
        Remote = 2//远程模式(暂未用)
    };

    /// <summary>
    /// 坐标类型枚举
    /// </summary>
    enum CsType : uint16_t
    {
        Jcs = 0,//关节空间坐标系
        Wcs = 1//笛卡尔空间坐标系(以机器人基坐标系为准)
    };

    /// <summary>
    /// 变量作用域枚举
    /// </summary>
    enum ScopeType : uint16_t
    {
        Local = 0,//局部变量
        Global = 1//全局变量
    };

    /// <summary>
    /// 变量类型枚举
    /// </summary>
    enum VarType : uint16_t
    {
        LJ = 0,//局部关节坐标
        LP = 1,//局部空间坐标
        VJ = 2,//局部关节运动速度
        VL = 3,//局部直线运动速度
        DM = 4,//局部整型
        DR = 5,//局部浮点型
        GJ = 6,//全局关节坐标
        GP = 7,//全局空间坐标
        M = 8,//全局布尔型
        EM = 9,//全局整型
        ER = 10//全局浮点型
    };

    /// <summary>
    /// IO类型枚举
    /// </summary>
    enum IOType : uint16_t
    {
        DI = 0,//数据输入
        DO = 1,//数字输出
        AI = 2,//模拟输入
        AO = 3,//模拟输出
        SIM_DI = 11,
        SIM_DO = 12,
        SIM_AI = 13,
        SIM_AO = 14
    };

    /// <summary>
    /// modbus操作量地址枚举
    /// ————————————————————————————————————————————
    /// 已知int是32位
    /// 取byte地址存入int的高16位
    /// 取bit地址存入int的低16位(MB没有bit地址默认用65535)
    /// 如：MX123.7 记为 （123<<16 | 7）
    /// 如：MB234   记为 （234<<16 | 65535）
    /// </summary>
    enum Address : int32_t
    {
        PlaceHolder = 0,

        VarGB = 50000 << 16 | 65535,//全局布尔变量 (bool[200]-8)
        VarGI = 50200 << 16 | 65535,//全局整形变量 (int[200]-16)
        VarGF = 50600 << 16 | 65535,//全局浮点变量 (float[200]-32)

        DigIn = 20000 << 16 | 65535,//数字输入端口 (bool[1000]-8)
        DigOut = 21000 << 16 | 65535,//数字输出端口 (bool[1000]-8)

        OpMode = 500 << 16 | 65535,//操作模式 (int-16) 0:手动模式；1:自动模式
        AutoStatus = 502 << 16 | 65535,//自动运行状态 (uint-16) 0:停止；1:启动；2:暂停；3:运行状态；4:暂停状态；5:停止状态
        AutoMode = 512 << 16 | 65535,//自动运行方式 (uint-16) 0:循环；1:单次；3:单步
        AutoStart = 8 << 16 | 0,//自动启动 (bit-1)
        AutoPause = 8 << 16 | 1,//自动暂停 (bit-1)
        AutoStop = 8 << 16 | 4,//自动停止 (bit-1)

        PowerType = 550 << 16 | 65535,//伺服使能类型 (int-16) 0:实体开关；1:虚拟开关
        PowerSignal = 10 << 16 | 0,//使能触发信号 (bit-1)
        PowerOk = 10 << 16 | 1,//使能OK (bit-1)

        SpeedGet = 510 << 16 | 65535,//全局速度值 (int-16)
        SpeedSet = 10 << 16 | 6,//速度赋值信号 (bit-1)
        SpeedPlus = 10 << 16 | 4,//速度+ (bit-1)
        SpeedMinus = 10 << 16 | 5,//速度- (bit-1)

        JogPlus = 20 << 16 | 65535,//Jog+ (bool[9]-8)
        JogMinus = 30 << 16 | 65535,//Jog- (bool[9]-8)
        AxesHomeSyn = 6 << 16 | 1,//全轴同步回零触发信号 (bit-1)
        AxesHomeAsyn = 6 << 16 | 2,//全轴依次回零触发信号 (bit-1)
        AxisHomeOk = 50 << 16 | 65535,//单轴回零Ok (bool[9]-8)
        AxesHomeOk = 6 << 16 | 5,//全轴回零ok (bit-1)

        CsType = 508 << 16 | 65535,//坐标系类型 (int-16)
        CurrentPos = 1900 << 16 | 65535,//当前坐标 (float[9]-32)
        CurrentPosJ = 4892 << 16 | 65535,//当前关节坐标 (float[9]-32)
        CurrentPosC = 4928 << 16 | 65535,//当前笛卡尔坐标 (float[9]-32)

        RemotePosSel = 800 << 16 | 65535,//远程坐标点选择 (int-16)
        RemotePosWo = 4820 << 16 | 65535,//远程坐标点只写 (float[9]-32) 
        RemotePosRo = 4856 << 16 | 65535,//远程坐标点只读 (float[9]-32)
        RemotePosSave = 16 << 16 | 0,//远程坐标点记录 (bit-1)
        RemotePosLoc = 16 << 16 | 1,//远程坐标点定位 (bit-1)
        RemotePosMod = 16 << 16 | 2,//远程坐标点修改 (bit-1)

        TaskNumNow = 2062 << 16 | 65535,//文件当前序号 (int-16)
        TaskNumExe = 2064 << 16 | 65535,//文件执行序号 (int-16) 执行读取操作时，当前文件序号=执行文件序号
        TaskNumObj = 2070 << 16 | 65535,//文件目标序号 (int-16) 执行复制操作时，目标文件的序号
        TaskNumMod = 105 << 16 | 2,//文件序号更改信号 (bit-1) 1:将当前文件序号更改为输入目标值
        TaskType = 2068 << 16 | 65535,//文件操作方式 (int-16) 0:读；1:写；2:删；3:复制；4:导出；5:导入
        TaskSignal = 104 << 16 | 0,//文件操作执行信号 (bit-1) 1:触发对选定文件执行选定操作方式
        TaskSave = 11 << 16 | 0,//文件保存 (int-16) 1:触发保存当前用户程序文件或系统参数文件
        LineNum = 2122 << 16 | 65535,//文件当前行号 (int-16) 从0开始
        LineNumSignal = 100 << 16 | 0,//文件行号更改信号 (bit-1) 1:将当前文件行号更改为目标输入值

        AlarmSign = 8 << 16 | 5,//警报标志 (bit-1) 0:无报警；1:有报警
        AlarmCode = 524 << 16 | 65535,//警报编号 (int-16)
        AlarmReset = 8 << 16 | 2//清错复位 (bit-1)
    };
}