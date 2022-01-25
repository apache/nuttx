/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_demo_strategy.c
 *
 * @brief
 *
 * @author
 *
 */

/* includes */
#include "stdint.h"
#include "string.h"
#include "gh3x2x_drv.h"
#include "gh3x2x_config.h"
#include "gh3x2x_inner.h"
#include "gh3x2x.h"


GU8 g_uchPreHrValue;


void Gh3x2xOutputValueStrategyInit(GU32 unFunctionID)
{
#if __GH3X2X_HR_OUTPUT_VALUE_STRATEGY_EN__
    if(GH3X2X_FUNCTION_HR & unFunctionID == GH3X2X_FUNCTION_HR)
    {
        g_uchPreHrValue = 0;
    }
#endif
}


#define GH3X2X_DIFF_ABS(x,y)  (x > y ? (x - y):(y - x))
#if __GH3X2X_HR_OUTPUT_VALUE_STRATEGY_EN__
void Gh3x2xHrOutputValueStrategyPro(STGh3x2xAlgoResult * pstAlgoResult, GU32 lubFrameId)
{
    GU8 uchCurrentOriginalHrValue = (GU8)(pstAlgoResult->snResult[0]);
    GU8 uchConf = (GU8)(pstAlgoResult->snResult[1]);
    GU8 uchNewHrValue = uchCurrentOriginalHrValue;

    if(0 == g_uchPreHrValue)
    {
        g_uchPreHrValue = uchCurrentOriginalHrValue;
    }
    else
    {
        if((GH3X2X_DIFF_ABS(g_uchPreHrValue,uchCurrentOriginalHrValue) > 50) &&(uchConf <= 50))   //there is big difference between pre hr and current orignal hr AND confidence is low
        {
            //smooth vaule
            uchNewHrValue = (((GU16)g_uchPreHrValue) *2 + (GU16)uchCurrentOriginalHrValue)/3;
        }
        g_uchPreHrValue = uchNewHrValue;

        //write back to result
        pstAlgoResult->snResult[0] = uchNewHrValue;
    }
}
#endif






















