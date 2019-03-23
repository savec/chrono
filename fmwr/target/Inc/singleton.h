#ifndef SINGLETON__
#define SINGLETON__

template <typename T>
class Singleton
{
    public:
        static T& get_instance()
        {
            static T instance;
            return instance;
        }
    protected:
        Singleton() {}
        ~Singleton() {}
    public:
        Singleton(Singleton const &) = delete;
        Singleton& operator=(Singleton const &) = delete;
};

#endif /* SINGLETON__ */
