#include "../Utils/maininclude.h"
#include <string>
#include <fstream>
#include <sys/time.h>
#include <pthread.h>
#include "y_orienter.h"
#include "visualiser.h"

//!!!!! mutex is not  locking in opengl program -> float calculations get corrupted


//#define ORIENTER_VISU
//reading takes 10 ms for 5 packets = 5*2ms with 500hz, orienter take much less than 10ms like big oozappo 100us ka na so total is 10ms for this configuration
#ifdef ORIENTER_VISU
    pthread_t thread_visu;
    pthread_mutex_t mu_visu;

void* handleVisu(void* p){

    Orienter* sp= (Orienter*)p;

    OrienterVisu visu;
    visu.init();
    visu.run(&mu_visu,&(sp->res_roll),&(sp->res_pitch),&(sp->res_yaw),&(sp->isupdated_marg),&(sp->x),&(sp->y),&(sp->z));
    pthread_exit(NULL);

}
#endif

int main(int argc, char **argv)
{

  ros::init(argc, argv, "orienter");
  ros::NodeHandle n;
  Orienter orienter(n);

    //Visualization
#ifdef ORIENTER_VISU
    pthread_mutex_init(&mu_visu,NULL);
    pthread_create(&thread_visu, NULL, handleVisu, (void*)&orienter);
#endif

  orienter.init();

  //ros::Rate loop_rate(YP_Orienter_looptime);
  while(ros::ok()) {
//cerr<<&mu_visu<<"locking orienter"<<endl;

#ifdef ORIENTER_VISU
pthread_mutex_lock(&mu_visu);
#endif


    orienter.receive();
    //orienter.marg_message_in.print();

    orienter.update();

    orienter.send();


   // usleep(1000000);

#ifdef ORIENTER_VISU
    pthread_mutex_unlock(&mu_visu);
#endif
  //  cerr<<"&mu_visu"<<"unlocking orienter"<<endl;

  }


#ifdef ORIENTER_VISU
  pthread_join(thread_visu,NULL);
  pthread_mutex_destroy(&mu_visu);
#endif
  return 0;
}



