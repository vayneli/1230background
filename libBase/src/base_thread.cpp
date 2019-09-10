#include "base_thread.h"  


void* BaseThread::run1()  
{  
    threadStatus = THREAD_STATUS_RUNNING;  
    tid = pthread_self();  
    run();  
    threadStatus = THREAD_STATUS_EXIT;  
    tid = 0;  
    pthread_exit(NULL);  
}  
  
BaseThread::BaseThread()  
{  
    tid = 0;  
    threadStatus = THREAD_STATUS_NEW;  
}  
  
bool BaseThread::start()  
{  
    int iRet = 0;  
    return  pthread_create(&tid, NULL, thread_proxy_func, this) == 0;  
}  
  
pthread_t BaseThread::getThreadID()  
{  
    return tid;  
}  
  
int BaseThread::getState()  
{  
    return threadStatus;  
}  
  
void BaseThread::join()  
{  
    if (tid > 0)  
    {  
        pthread_join(tid, NULL);  
    }  
}  
void * BaseThread::thread_proxy_func(void * args)  
{  
        BaseThread * pThread = static_cast<BaseThread *>(args);   
   
        pThread->run1();
        return pThread;
}  
  
void BaseThread::join(unsigned long millisTime)  
{  
    if (tid == 0)  
    {  
        return;  
    }  
    if (millisTime == 0)  
    {  
        join();  
    }else{
        unsigned long k = 0;  
        while (threadStatus != THREAD_STATUS_EXIT && k <= millisTime)  
        {  
            usleep(100); 
            k++;  
        }  
    }  
}  
