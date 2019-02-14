//
// Created by rfeldhans on 08.02.19.
//

#include "nodes/utils.h"
#include <stdexcept>
#include <boost/algorithm/string/predicate.hpp>

namespace esiaf_ros {
    namespace utils {

        // Update the input string.
        void autoExpandEnvironmentVariables(std::string &text) {
            static std::regex env("\\$\\{([^}]+)\\}");
            std::smatch match;
            while (std::regex_search(text, match, env)) {
                const char *s = getenv(match[1].str().c_str());
                const std::string var(s == NULL ? "" : s);
                text.replace(match[0].first, match[0].second, var);
            }
        }


        esiaf_ros::Bitrate cfg_to_esiaf_bitrate(int bitrate, char signed_value, char type_value) {
            if(!(signed_value == 's' || signed_value == 'u')){
                std::string ex_text = " signed value of %c, but only s (signed) and u (unsigned) are allowed at this point!", signed_value;
                throw std::invalid_argument(ex_text);
            }
            if(!(type_value == 'i' || type_value == 'f')){
                std::string ex_text = " type value of %c, but only i (int) and f (float) are allowed at this point!", type_value;
                throw std::invalid_argument(ex_text);
            }
            switch (bitrate) {
                case 8 :
                    if (signed_value == 's'){
                        return esiaf_ros::Bitrate::BIT_INT_8_SIGNED;
                    } else{
                        return esiaf_ros::Bitrate::BIT_INT_8_UNSIGNED;
                    }
                case 16 :
                    if (signed_value == 's'){
                        return esiaf_ros::Bitrate::BIT_INT_16_SIGNED;
                    } else{
                        return esiaf_ros::Bitrate::BIT_INT_16_UNSIGNED;
                    }
                case 24 :
                    if (signed_value == 's'){
                        return esiaf_ros::Bitrate::BIT_INT_24_SIGNED;
                    } else{
                        return esiaf_ros::Bitrate::BIT_INT_24_UNSIGNED;
                    }
                case 32 :
                    if (type_value == 'f'){
                        return esiaf_ros::Bitrate::BIT_FLOAT_32;
                    }
                    if (signed_value == 's'){
                        return esiaf_ros::Bitrate::BIT_INT_32_SIGNED;
                    } else{
                        return esiaf_ros::Bitrate::BIT_INT_32_UNSIGNED;
                    }
                case 64:
                    return esiaf_ros::Bitrate::BIT_FLOAT_64;
                default:
                    std::string ex_text = " received %d, but only 8, 16, 24, 32 and 64 bit are allowed at this point!", bitrate;
                    throw std::invalid_argument(ex_text);
            }
        }

        esiaf_ros::Endian cfg_to_esiaf_endian(std::string endian){
            if(boost::iequals(endian, "LittleEndian") || boost::iequals(endian, "LE") ){
                return esiaf_ros::Endian::LittleEndian;
            }
            if(boost::iequals(endian, "BigEndian") || boost::iequals(endian, "BE") ){
                return esiaf_ros::Endian::BigEndian;
            }
            std::string ex_text = (" received %s, but only LitteEndian and BigEndian are allowed!", endian);
            throw std::invalid_argument(ex_text);
        }

        esiaf_ros::Rate cfg_to_esaif_rate(int rate) {
            switch (rate) {
                case 8 :
                case 8000 :
                    return esiaf_ros::Rate::RATE_8000;
                case 16 :
                case 16000 :
                    return esiaf_ros::Rate::RATE_16000;
                case 32 :
                case 32000 :
                    return esiaf_ros::Rate::RATE_32000;
                case 44 :
                case 441 :
                case 44100 :
                    return esiaf_ros::Rate::RATE_44100;
                case 48 :
                case 48000 :
                    return esiaf_ros::Rate::RATE_48000;
                case 96 :
                case 96000 :
                    return esiaf_ros::Rate::RATE_96000;
                default:
                    std::string ex_text = " received %d, but only 8k, 16k, 32k, 441000, 48k and 96k Hz are allowed at this point!", rate;
                    throw std::invalid_argument(ex_text);
            }
        }


        _snd_pcm_format set_up_format_from_bitrate_and_endian(esiaf_ros::Bitrate bitrate, esiaf_ros::Endian endian) {
            switch (bitrate) {
                case esiaf_ros::Bitrate::BIT_INT_8_SIGNED :
                    return SND_PCM_FORMAT_S8;
                case esiaf_ros::Bitrate::BIT_INT_8_UNSIGNED :
                    return SND_PCM_FORMAT_U8;
                case esiaf_ros::Bitrate::BIT_INT_16_SIGNED :
                    switch (endian) {
                        case esiaf_ros::Endian::LittleEndian:
                            return SND_PCM_FORMAT_S16_LE;
                        case esiaf_ros::Endian::BigEndian:
                            return SND_PCM_FORMAT_S16_BE;
                    }
                case esiaf_ros::Bitrate::BIT_INT_16_UNSIGNED :
                    switch (endian) {
                        case esiaf_ros::Endian::LittleEndian:
                            return SND_PCM_FORMAT_U16_LE;
                        case esiaf_ros::Endian::BigEndian:
                            return SND_PCM_FORMAT_U16_BE;
                    }
                case esiaf_ros::Bitrate::BIT_INT_24_SIGNED :
                    switch (endian) {
                        case esiaf_ros::Endian::LittleEndian:
                            return SND_PCM_FORMAT_S24_LE;
                        case esiaf_ros::Endian::BigEndian:
                            return SND_PCM_FORMAT_S24_BE;
                    }
                case esiaf_ros::Bitrate::BIT_INT_24_UNSIGNED :
                    switch (endian) {
                        case esiaf_ros::Endian::LittleEndian:
                            return SND_PCM_FORMAT_U24_LE;
                        case esiaf_ros::Endian::BigEndian:
                            return SND_PCM_FORMAT_U24_BE;
                    }
                case esiaf_ros::Bitrate::BIT_INT_32_SIGNED :
                    switch (endian) {
                        case esiaf_ros::Endian::LittleEndian:
                            return SND_PCM_FORMAT_S32_LE;
                        case esiaf_ros::Endian::BigEndian:
                            return SND_PCM_FORMAT_S32_BE;
                    }
                case esiaf_ros::Bitrate::BIT_INT_32_UNSIGNED :
                    switch (endian) {
                        case esiaf_ros::Endian::LittleEndian:
                            return SND_PCM_FORMAT_U32_LE;
                        case esiaf_ros::Endian::BigEndian:
                            return SND_PCM_FORMAT_U32_BE;
                    }
                case esiaf_ros::Bitrate::BIT_FLOAT_32 :
                    return SND_PCM_FORMAT_FLOAT;
                case esiaf_ros::Bitrate::BIT_FLOAT_64 :
                    return SND_PCM_FORMAT_FLOAT64;
                default:
                    return SND_PCM_FORMAT_UNKNOWN;
            }
        }

        unsigned int set_up_sample_rate_from_esiaf(esiaf_ros::Rate sample_rate) {
            switch (sample_rate) {
                case esiaf_ros::Rate::RATE_8000:
                    return 8000;
                case esiaf_ros::Rate::RATE_16000:
                    return 16000;
                case esiaf_ros::Rate::RATE_32000:
                    return 32000;
                case esiaf_ros::Rate::RATE_44100:
                    return 44100;
                case esiaf_ros::Rate::RATE_48000:
                    return 48000;
                case esiaf_ros::Rate::RATE_96000:
                    return 96000;
            }
        }

    }
}

