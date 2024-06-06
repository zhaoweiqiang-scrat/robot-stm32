#ifdef __cplusplus
extern "C" {
#endif
/************************************************************************************
* @fuction name：FUZZY_PID_CONTROL
* @fuction description： 模糊自适应控制算法，为了方便测试默认e、ec在[-3,3]区间，
* 如需改变e、ec范围，需引入量化因子(Ke、Kec=N/emax)、缩放因子(Ku=umax/N)。以下代码采
*用三角隶属函数求隶属度以及加权平均法解模糊，PID采用增量式PID算法，算法仅供参考，欢迎报错。
*************************************************************************************/

// 学习网址： https://blog.csdn.net/shuoyueqishilove/article/details/78236541
//			  https://blog.csdn.net/a841771798/article/details/79323118

#include "fuzzypid .h"
#include "math.h"


F_PID fuzzy(float e,float ec) // e 是目标值和反馈值的误差 ec是误差变化率(误差e的微分) 
{
     float etemp,ectemp;
     float eLefttemp,ecLefttemp;
     float eRighttemp ,ecRighttemp;

     int eLeftIndex,ecLeftIndex;
     int eRightIndex,ecRightIndex;
     F_PID      fuzzy_PID;
	
     etemp = e > 3.0 ? 0.0 : (e < - 3.0 ? 0.0 : (e >= 0.0 ? (e >= 2.0 ? 2.5: (e >= 1.0 ? 1.5 : 0.5)) : (e >= -1.0 ? -0.5 : (e >= -2.0 ? -1.5 : (e >= -3.0 ? -2.5 : 0.0) ))));

     eLeftIndex = (int)e;
     eRightIndex = eLeftIndex;
     eLeftIndex = (int)((etemp-0.5) + 3);        //[-3,3] -> [0,6]
     eRightIndex = (int)((etemp+0.5) + 3);

     eLefttemp =etemp == 0.0 ? 0.0:((etemp+0.5)-e);
     eRighttemp=etemp == 0.0 ? 0.0:( e-(etemp-0.5));

     ectemp = ec > 3.0 ? 0.0 : (ec < - 3.0 ? 0.0 : (ec >= 0.0 ? (ec >= 2.0 ? 2.5: (ec >= 1.0 ? 1.5 : 0.5)) : (ec >= -1.0 ? -0.5 : (ec >= -2.0 ? -1.5 : (ec >= -3.0 ? -2.5 : 0.0) ))));

     ecLeftIndex = (int)((ectemp-0.5) + 3);        //[-3,3] -> [0,6]
     ecRightIndex = (int)((ectemp+0.5) + 3);

     ecLefttemp =ectemp == 0.0 ? 0.0:((ectemp+0.5)-ec);
     ecRighttemp=ectemp == 0.0 ? 0.0:( ec-(ectemp-0.5));

/*************************************反模糊*************************************/
	fuzzy_PID.Kp = (eLefttemp * ecLefttemp *  fuzzyRuleKp[ecLeftIndex][eLeftIndex]
					+ eLefttemp * ecRighttemp * fuzzyRuleKp[ecRightIndex][eLeftIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKp[ecLeftIndex][eRightIndex]
					+ eRighttemp * ecRighttemp * fuzzyRuleKp[ecRightIndex][eRightIndex]);

	fuzzy_PID.Ki =   (eLefttemp * ecLefttemp * fuzzyRuleKi[ecLeftIndex][eLeftIndex]
					+ eLefttemp * ecRighttemp * fuzzyRuleKi[ecRightIndex][eLeftIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKi[ecLeftIndex][eRightIndex]
					+ eRighttemp * ecRighttemp * fuzzyRuleKi[ecRightIndex][eRightIndex]);

	fuzzy_PID.Kd = (eLefttemp * ecLefttemp *    fuzzyRuleKd[ecLeftIndex][eLeftIndex]
					+ eLefttemp * ecRighttemp * fuzzyRuleKd[ecRightIndex][eLeftIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKd[ecLeftIndex][eRightIndex]
					+ eRighttemp * ecRighttemp * fuzzyRuleKd[ecRightIndex][eRightIndex]);
	return fuzzy_PID;

}


float FuzzyPid_Out(float tar,float cur)  // 目标值 , 实际值
{
    //float tar = 0,cur = 0;                //目标值 , 实际值
    float e = 0;       // 误差e 误差变化率ec(误差e的微分)  系统死区设置量 不一定为零
//	static PID pid= {0.6, 0.082, 0.12};      //赋予初值kp，ki，kd
	//static F_PID pid= {12.0,0.06,-2.8};
	static F_PID pid= {0.05,0.02,0.022};
	static float sumOUT = 0,pwmOUT=0,old1 = 0,old2 = 0,ec=0;                   //累加偏差 old1 old2
	F_PID OUT = {0, 0, 0};
	e = cur-tar;             //实际值 - 目标值
	ec = e-old1;            
	OUT = fuzzy(e, ec);      //模糊控制调整  kp，ki，kd

	pwmOUT = (pid.Kp+OUT.Kp)*e+(pid.Ki+OUT.Ki)*old1+(pid.Kd+OUT.Kd)*old2;
	old2= old1;
	old1 = e;
	sumOUT+=pwmOUT;
	return sumOUT; //最终输出值
}
#ifdef __cplusplus
}
#endif