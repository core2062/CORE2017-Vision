#pragma once

#include <vector>
#include <string>
#include <iostream>

using namespace std;

namespace CORE {
    class ICOREArgument {
    public:
        virtual ~ICOREArgument() {}
        string getHelpText();
        virtual bool updateArgument(char* argv[]) = 0;
    protected:
        string m_argumentName;
        string m_helpText;
    };

    class COREArgumentManager {
    public:
        static void updateArguments(int argc, char* argv[]);
        static void addArgument(ICOREArgument * argument);
        static void cleanUp();
    private:
        static vector<ICOREArgument*> m_arguments;
    };

    template<class T>
    class COREArgument : public ICOREArgument {
    public:
        COREArgument(string argumentName, string helpText, T defaultValue) {
            m_argumentName = argumentName;
            m_helpText = helpText;
            m_value = defaultValue;
        }

        bool updateArgument(char* argv[]) {

        }

        T Get() {
            return m_value;
        }

    private:
        T m_value;
    };
}

#include "COREArgument.inc"