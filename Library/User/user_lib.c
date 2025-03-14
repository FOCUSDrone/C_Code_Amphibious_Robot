#include "user_lib.h"
#include "arm_math.h"

/**
 * @brief:       invSqrt: ���ٿ�������
 * @param[in]:   num: ����ĸ�����
 * @retval:      ������������ƽ�����ĵ���
 * @details:     ʹ�ÿ��ٿ����㷨����ħ�����������㸡������ƽ����������
 */
fp32 invSqrt(fp32 num)
{
    fp32 halfnum = 0.5f * num;
    fp32 y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(fp32 *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
 * @brief:       ramp_init: б��������ʼ��
 * @param[in]:   ramp_source_type: б�������ṹ��
 * @param[in]:   frame_period: ���ʱ�䣨��λ���룩
 * @param[in]:   max: ���ֵ
 * @param[in]:   min: ��Сֵ
 * @retval:      ���ؿ�
 * @details:     ��ʼ��б�������Ľṹ�壬����б����ʱ���������ֵ����Сֵ�Լ���ʼֵ��
 */
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
 * @brief:       ramp_calc: б����������
 * @param[in]:   ramp_source_type: б�������ṹ��
 * @param[in]:   input: ����ֵ����λ��/s����ÿ�����ӵ�ֵ��
 * @retval:      ���ؿ�
 * @details:     ���������ֵ���е��ӣ����㵱ǰб�����������ֵ�������ָ�������ֵ����Сֵ֮�䡣
 */
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}

/**
 * @brief:       first_order_filter_init: һ�׵�ͨ�˲���ʼ��
 * @param[in]:   first_order_filter_type: һ�׵�ͨ�˲��ṹ��
 * @param[in]:   frame_period: ���ʱ�䣨��λ���룩
 * @param[in]:   num: �˲�����
 * @retval:      ���ؿ�
 * @details:     ��ʼ��һ�׵�ͨ�˲��ṹ�壬�����˲���ʱ�������˲�������
 */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
 * @brief:       first_order_filter_cali: һ�׵�ͨ�˲�����
 * @param[in]:   first_order_filter_type: һ�׵�ͨ�˲��ṹ��
 * @param[in]:   input: ����ֵ����λ���룩
 * @retval:      ���ؿ�
 * @details:     ������ֵ����һ�׵�ͨ�˲���������˲���Ľ����
 */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] +
        first_order_filter_type->frame_period) * first_order_filter_type->out +
        first_order_filter_type->frame_period / (first_order_filter_type->num[0] +
        first_order_filter_type->frame_period) * first_order_filter_type->input;
}

/**
 * @brief:       abs_limit: ��������
 * @param[in/out]: num: ����/���ֵ���������ƺ󷵻�
 * @param[in]:   Limit: ����ֵ
 * @retval:      ���ؿ�
 * @details:     ������ֵ���о���ֵ���ƣ�����������ָ���ķ�Χ�ڡ�
 */
void abs_limit(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

/**
 * @brief:       sign: �жϷ���λ
 * @param[in]:   value: ����ֵ
 * @retval:      ���ط��ţ��������� 1���������� -1���㷵�� 1
 * @details:     �ж�����ֵ�ķ���λ������ 1 �� -1��
 */
fp32 sign(fp32 value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

/**
 * @brief:       fp32_deadline: ����������
 * @param[in]:   Value: ����ĸ�����
 * @param[in]:   minValue: ��������
 * @param[in]:   maxValue: ��������
 * @retval:      ���ش�����ֵ��������ֵ�������ڣ����� 0
 * @details:     ������ĸ������������������������ָ����Χ�ڣ��򷵻� 0��
 */
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

/**
 * @brief:       int16_deadline: int16����������
 * @param[in]:   Value: ���������
 * @param[in]:   minValue: ��������
 * @param[in]:   maxValue: ��������
 * @retval:      ���ش�����������������ֵ�������ڣ����� 0
 * @details:     ������������������������������ָ����Χ�ڣ��򷵻� 0��
 */
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

/**
 * @brief:       fp32_constrain: �޷������������ͣ�
 * @param[in]:   Value: ����ֵ
 * @param[in]:   minValue: ��Сֵ
 * @param[in]:   maxValue: ���ֵ
 * @retval:      �޷���Ľ��
 * @details:     ������ֵ�����޷�����������ָ������Сֵ�����ֵ֮�䡣
 */
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

/**
 * @brief:       int16_constrain: �޷������������ͣ�
 * @param[in]:   Value: ����ֵ
 * @param[in]:   minValue: ��Сֵ
 * @param[in]:   maxValue: ���ֵ
 * @retval:      �޷���Ľ��
 * @details:     �����������ֵ�����޷�����������ָ������Сֵ�����ֵ֮�䡣
 */
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

/**
 * @brief:       loop_fp32_constrain: ѭ���޷������������ͣ�
 * @param[in]:   Input: ����ֵ
 * @param[in]:   minValue: ��Сֵ
 * @param[in]:   maxValue: ���ֵ
 * @retval:      �޷���Ľ��
 * @details:     ������ĸ���������ѭ���޷�����ʹ���볬����Χ��Ҳ��ѭ���ص�ָ����Χ�ڡ�
 */
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

/**
 * @brief:       theta_format: �Ƕȸ�ʽ��Ϊ -180 ~ 180
 * @param[in]:   Ang: ����Ƕ�
 * @retval:      ��ʽ����ĽǶ�
 * @details:     ������Ƕȸ�ʽ��Ϊ -180 �� 180 ��֮�䡣
 */
fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}

/**
 * @brief:       rad_format: �Ƕȸ�ʽ��Ϊ -180 ~ 180
 * @param[in]:   Ang: ����Ƕ�
 * @retval:      ��ʽ����ĽǶ�
 * @details:     ������Ƕȸ�ʽ��Ϊ -180 �� 180 ��֮�䡣
 */
fp32 rad_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -PI, PI);
}

/**
 * @brief:       float_to_uint: ������ת��Ϊ�޷�����������
 * @param[in]:   x_float:��ת���ĸ�����
 * @param[in]:   x_min:��Χ��Сֵ
 * @param[in]:   x_max:��Χ���ֵ
 * @param[in]:   bits:Ŀ���޷���������λ��
 * @retval:      �޷����������
 * @details:     �������ĸ����� x ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ��ָ��λ�����޷���������
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief:       uint_to_float: �޷�������ת��Ϊ����������
 * @param[in]:   x_int: ��ת�����޷�������
 * @param[in]:   x_min: ��Χ��Сֵ
 * @param[in]:   x_max: ��Χ���ֵ
 * @param[in]:   bits:  �޷���������λ��
 * @retval:      ���������
 * @details:     ���������޷������� x_int ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ����������
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int16_t int16_abs(int16_t val)
{
    if (val < 0){
        return -val;
    }
    else return val;
}

