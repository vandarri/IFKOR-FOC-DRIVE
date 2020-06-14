/**
 * @file pmsm_foc_vq_ctrl.c
 * @date 2015-12-23
 *
 * @cond
 *********************************************************************************************************************
 * PMSM FOC Motor Control Library
 *
 * Copyright (c) 2015-2016, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes
 * with Infineon Technologies AG (dave@infineon.com).
 ***********************************************************************************************************************
 *
 * @file pmsm_foc_vq_ctrl.c
 * @date 23 Dec, 2015
 * @version 1.0.0
 *
 * @brief ACMP low level driver API prototype definition for XMC1 <br>
 *
 * <b>Detailed description of file</b> <br>
 * APIs provided in this file mainly cover the following functionality:
 * ---- Filter, Hysterisis, Output inversion
 *
 * History
 *
 * 23 Dec 2015 Version 1.0.0 <br>:
 *      Initial version
 * @endcond
 *
 */

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "pmsm_foc_vq_ctrl.h"

/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/
 /* ADC results, trigger positions.*/
extern ADCType ADC;
/* Motor control information */
extern MotorControlType Motor;
/* SVM information, such as sector 0 ~ 5 (A ~ F) in SVM space vector hexagon.*/
extern SVMType SVM;
/* Speed PI controller.*/
extern PI_Coefs_Type PI_Speed;
/* Torque / Iq PI controller.*/
extern PI_Coefs_Type PI_Torque;
/* Flux /Id PI controller. */
extern PI_Coefs_Type PI_Flux;
extern uint32_t * NEW_SHS0_CALOC1;
extern Car2PolType Car2Polar;
extern PLL_EstimatorType PLL_Estimator;
extern ClarkeTransformType Clarke_Transform;
extern ParkTransformType Park_Transform;
/* PLL rotor speed PI controller. */
extern PI_Coefs_Type PI_PLL;
/* Parameters input for FOC LIB.*/
extern FOCInputType FOCInput;
/* Output for FOC LIB */
extern FOCOutputType FOCOutput;
/*  Motor current and current space vector.*/
extern CurrentType Current;

/* CORDIC: Circular Vectoring Mode (default). MPS: Divide by 2 (default).*/
#define CORDIC_VECTORING_MODE   (0x62)
/* CORDIC: Circular Rotation Mode. MPS: Divide by 2 (default) */
#define CORDIC_ROTATION_MODE    (0x6A)


#if(MY_FOC_CONTROL_SCHEME == CONSTANT_VQ_DIRECT_FOC)
 __attribute__((section(".ram_code")))  void FOC_VQ_Controller (void)
{
#if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
 // Get_ADC_SingleShuntCurrent(&ADC);
  Current_Reconstruction(FOCOutput.Previous_SVM_SectorNo, ADC.ADC_Result1, ADC.ADC_Result2, &Current);

#else
  Get_ADCPhaseCurrent(FOCOutput.Previous_SVM_SectorNo, FOCOutput.New_SVM_SectorNo, &ADC);

  Current_Reconstruction(ADC.ADC_Iu, ADC.ADC_Iv, ADC.ADC_Iw, &Current);
#endif
  Motor.Speed = PLL_Estimator.RotorSpeed_In;



  /* Motor reference speed. */
  FOCInput.Ref_Speed = Motor.Ref_Speed;

  ClarkeTransform(Current.I_U, Current.I_V, Current.I_W, &Clarke_Transform);

  ParkTransform(Clarke_Transform.I_Alpha_1Q31, Clarke_Transform.I_Beta_1Q31, PLL_Estimator.RotorAngleQ31);

  /* Record previous SVM sector number. */
  FOCOutput.Previous_SVM_SectorNo = FOCOutput.New_SVM_SectorNo;

  ParkTransform_GetResult(&Park_Transform);

  PLL_Imag(Car2Polar.Vref_AngleQ31, Clarke_Transform.I_Alpha_1Q31,Clarke_Transform.I_Beta_1Q31);

  Car2Polar.Torque_Vq = FOCInput.Vq;

#if(OVERCURRENT_PROTECTION == ENABLED)
  Over_current_Protection_check(ADC.ADC_IDCLink, Park_Transform.Iq, &FOCInput.overcurrent_factor);
  Car2Polar.Torque_Vq = (Car2Polar.Torque_Vq * FOCInput.overcurrent_factor) >> 12;        // Motor reference speed.
#endif

  PLL_Imag_GetResult(&PLL_Estimator);

  PLL_Vref(PLL_Estimator.Delta_IV, Car2Polar.Vref32, PI_PLL.Uk, FOCInput.Phase_L,&PLL_Estimator);

  /* PI Controller #3 - Flux / Id PI controller of FOC */
  PI_controller(FOCInput.Ref_Id, Park_Transform.Id, &PI_Flux);

  Car2Polar.Flux_Vd = PI_Flux.Uk;

  PLL_Vref_GetResult(&PLL_Estimator);

  Cart2Polar(Car2Polar.Torque_Vq, Car2Polar.Flux_Vd,PLL_Estimator.RotorAngleQ31);

  PLL_GetPosSpd(&PLL_Estimator);

  Car2Pol_GetResult(&Car2Polar);

  uint32_t SVM_Vref16;
  SVM_Vref16 = Car2Polar.Vref32 >>CORDIC_SHIFT;

  SVM_Vref16 = (SVM_Vref16 * 311) >> 8;
  Car2Polar.Vref32 = SVM_Vref16 << CORDIC_SHIFT;



}


__attribute__((section(".ram_code"))) void Linear_VQ_Ramp_Generator(int32_t current_set, int32_t inc_step,
                                                                    int32_t dec_step, FOCInputType* const HandlePtr)
{
    static uint32_t vq_counter = 0;

    vq_counter++;
    if(vq_counter >= USER_VQ_RAMP_SLEWRATE)
    {
        vq_counter = 0;
        if( HandlePtr->Vq < current_set)
        {
          HandlePtr->Vq += inc_step;
        }
        else if(HandlePtr->Vq > current_set)
        {
          if((HandlePtr->Vq >= dec_step )&& (ADC.ADC_DCLink < VDC_MAX_LIMIT))
          {
            HandlePtr->Vq -=  dec_step;
          }

        }
    }
  /* Limit protection for Vq, the max value is capped up to 1Q15 */
  if (HandlePtr->Vq > USER_VQ_REF_HIGH_LIMIT)
  {
    HandlePtr->Vq = USER_VQ_REF_HIGH_LIMIT - 1;
  }

}
#endif


