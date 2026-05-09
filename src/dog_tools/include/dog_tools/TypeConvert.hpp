#pragma once

#include <cstdint>

namespace Dog8Dof {
    /**
     * @brief 将浮点数映射到整型
     *
     * @param x 浮点数
     * @param Float_1 浮点数1
     * @param Float_2 浮点数2
     * @param Int_1 整型1
     * @param Int_2 整型2
     * @return int32_t 整型
     */
    inline int32_t Math_Float_To_Int(
        float x, float Float_1, float Float_2, int32_t Int_1, int32_t Int_2) {
        float tmp = (x - Float_1) / (Float_2 - Float_1);
        int32_t out = tmp * (float)(Int_2 - Int_1) + Int_1;
        return (out);
    }

    /**
     * @brief 将整型映射到浮点数
     *
     * @param x 整型
     * @param Int_1 整型1
     * @param Int_2 整型2
     * @param Float_1 浮点数1
     * @param Float_2 浮点数2
     * @return float 浮点数
     */
    inline float Math_Int_To_Float(
        int32_t x, int32_t Int_1, int32_t Int_2, float Float_1, float Float_2) {
        float tmp = (float)(x - Int_1) / (float)(Int_2 - Int_1);
        float out = tmp * (Float_2 - Float_1) + Float_1;
        return (out);
    }

    /**
     * @brief 16位大小端转换
     *
     * @param Address 地址
     */
    inline void Math_Endian_Reverse_16(void *Address) {
        uint8_t *temp_address_8 = (uint8_t *)Address;
        uint16_t *temp_address_16 = (uint16_t *)Address;
        *temp_address_16 = temp_address_8[0] << 8 | temp_address_8[1];
    }

    /**
     * @brief 16位大小端转换
     *
     * @param Source 源数据地址
     * @param Destination 目标存储地址
     * @return uint16_t 结果
     */
    inline uint16_t Math_Endian_Reverse_16(void *Source, void *Destination) {
        uint8_t *temp_address_8 = (uint8_t *)Source;
        uint16_t temp_address_16;
        temp_address_16 = temp_address_8[0] << 8 | temp_address_8[1];

        if (Destination != nullptr) {
            uint8_t *temp_source, *temp_destination;
            temp_source = (uint8_t *)Source;
            temp_destination = (uint8_t *)Destination;

            temp_destination[0] = temp_source[1];
            temp_destination[1] = temp_source[0];
        }

        return temp_address_16;
    }

    /**
     * @brief 数值限幅函数
     *
     * @param Value 指向被限幅的变量的指针
     * @param Min 最小值
     * @param Max 最大值
     */
    template <typename T> void Math_Constrain(T *Value, T Min, T Max) {
        if (*Value < Min) {
            *Value = Min;
        } else if (*Value > Max) {
            *Value = Max;
        }
    }

} // namespace Dog8Dof
