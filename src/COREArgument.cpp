#include "COREArgument.h"

using namespace CORE;

vector<ICOREArgument*> COREArgumentManager::m_arguments;

string ICOREArgument::getHelpText() {
    return m_helpText;
}

void COREArgumentManager::updateArguments(int argc, char* argv[]) {
    bool fail = false;
    for(auto argument : m_arguments) {
        fail = argument->updateArgument(argv);
    }
    if(fail) {
        cout << "Invalid argument(s) specified!" << endl;
        for(auto argument : m_arguments) {
            cout << argument->getHelpText() << endl;
        }
    }
}

void COREArgumentManager::addArgument(ICOREArgument * argument) {
    m_arguments.push_back(argument);
}

void COREArgumentManager::cleanUp() {
    for (auto i = m_arguments.begin(); i != m_arguments.end(); i++){
        delete *i;
    }
    m_arguments.clear();
}
