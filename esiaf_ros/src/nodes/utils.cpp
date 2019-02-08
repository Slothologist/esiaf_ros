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


        esiaf_ros::Bitrate cfg_to_esiaf_bitrate(int bitrate) {
            switch (bitrate) {
                case 8 :
                    return esiaf_ros::Bitrate::BIT_8;
                case 16 :
                    return esiaf_ros::Bitrate::BIT_16;
                case 24 :
                    return esiaf_ros::Bitrate::BIT_24;
                case 32 :
                    return esiaf_ros::Bitrate::BIT_32;
                default:
                    std::string ex_text = " received %d, but only 8, 16, 24 and 32 bit are allowed at this point!", bitrate;
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

    }
}

