#ifdef __cplusplus
extern "C" {
#endif
/************************************************************************************
* @fuction name��FUZZY_PID_CONTROL
* @fuction description�� ģ������Ӧ�����㷨��Ϊ�˷������Ĭ��e��ec��[-3,3]���䣬
* ����ı�e��ec��Χ����������������(Ke��Kec=N/emax)����������(Ku=umax/N)�����´����
*�����������������������Լ���Ȩƽ������ģ����PID��������ʽPID�㷨���㷨�����ο�����ӭ����
*************************************************************************************/

// ѧϰ��ַ�� https://blog.csdn.net/shuoyueqishilove/article/details/78236541
//			  https://blog.csdn.net/a841771798/article/details/79323118

#include "fuzzypid .h"
#include "math.h"


F_PID fuzzy(float e,float ec) // e ��Ŀ��ֵ�ͷ���ֵ����� ec�����仯��(���e��΢��) 
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

/*************************************��ģ��*************************************/
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


float FuzzyPid_Out(float tar,float cur)  // Ŀ��ֵ , ʵ��ֵ
{
    //float tar = 0,cur = 0;                //Ŀ��ֵ , ʵ��ֵ
    float e = 0;       // ���e ���仯��ec(���e��΢��)  ϵͳ���������� ��һ��Ϊ��
//	static PID pid= {0.6, 0.082, 0.12};      //�����ֵkp��ki��kd
	//static F_PID pid= {12.0,0.06,-2.8};
	static F_PID pid= {0.05,0.02,0.022};
	static float sumOUT = 0,pwmOUT=0,old1 = 0,old2 = 0,ec=0;                   //�ۼ�ƫ�� old1 old2
	F_PID OUT = {0, 0, 0};
	e = cur-tar;             //ʵ��ֵ - Ŀ��ֵ
	ec = e-old1;            
	OUT = fuzzy(e, ec);      //ģ�����Ƶ���  kp��ki��kd

	pwmOUT = (pid.Kp+OUT.Kp)*e+(pid.Ki+OUT.Ki)*old1+(pid.Kd+OUT.Kd)*old2;
	old2= old1;
	old1 = e;
	sumOUT+=pwmOUT;
	return sumOUT; //�������ֵ
}
#ifdef __cplusplus
}
#endif