namespace CORE {
    template<>
    class COREArgument<bool> : public ICOREArgument {
    public:
        COREArgument(string argumentName, string helpText, bool defaultValue) {
            m_argumentName = argumentName;
            m_helpText = helpText;
            m_value = defaultValue;
        }

        bool updateArgument(char* argv[]) {
            
        }

        bool Get() {
            return m_value;
        }

    private:
        bool m_value;
    };

    template<>
    class COREArgument<double> : public ICOREArgument {
    public:
        COREArgument(string argumentName, string helpText, double defaultValue) {
            m_argumentName = argumentName;
            m_helpText = helpText;
            m_value = defaultValue;
        }

        bool updateArgument(char* argv[]) {

        }

        double Get() {
            return m_value;
        }

    private:
        double m_value;
    };

    template<>
    class COREArgument<string> : public ICOREArgument {
    public:
        COREArgument(string argumentName, string helpText, string defaultValue) {
            m_argumentName = argumentName;
            m_helpText = helpText;
            m_value = defaultValue;
        }

        bool updateArgument(char* argv[]) {

        }

        string Get() {
            return m_value;
        }

    private:
        string m_value;
    };
}