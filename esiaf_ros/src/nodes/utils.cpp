//
// Created by rfeldhans on 08.02.19.
//

#include "nodes/utils.h"

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

    }
}

