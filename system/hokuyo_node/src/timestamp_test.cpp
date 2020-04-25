#include <hokuyo_node/hokuyo.h>
#include <time.h>
#include <sys/time.h> 
#include <algorithm>

static uint64_t timeHelper()
{
#if POSIX_TIMERS > 0
  struct timespec curtime;
  clock_gettime(CLOCK_REALTIME, &curtime);
  return (uint64_t)(curtime.tv_sec) * 1000000000 + (uint64_t)(curtime.tv_nsec);
#else
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  return (uint64_t)(timeofday.tv_sec) * 1000000000 + (uint64_t)(timeofday.tv_usec) * 1000;
#endif
}

template <class C>
C median(std::vector<C> &v)
{
  std::vector<long long int>::iterator start  = v.begin();
  std::vector<long long int>::iterator end    = v.end();
  std::vector<long long int>::iterator median = start + (end - start) / 2;
  //std::vector<long long int>::iterator quarter1 = median - (end - start) / 4;
  //std::vector<long long int>::iterator quarter2 = median + (end - start) / 4;
  std::nth_element(start, median, end);
  //long long int medianval = *median;
  //std::nth_element(start, quarter1, end);
  //long long int quarter1val = *quarter1;
  //std::nth_element(start, quarter2, end);
  //long long int quarter2val = *quarter2;
  return *median;
}

long long int getClockOffset(hokuyo::Laser &laser, int reps)
{
  std::vector<long long int> offset(reps);
  int timeout = 1000;

  laser.sendCmd("TM0",timeout);
  for (int i = 0; i < reps; i++)
  {
    long long int prestamp = timeHelper();
    laser.sendCmd("TM1",timeout);
    long long int hokuyostamp = laser.readTime();
    long long int poststamp = timeHelper();
    offset[i] = hokuyostamp - (prestamp + poststamp) / 2;
    printf("%lli %lli %lli - ", prestamp, hokuyostamp, poststamp);
  }
  laser.sendCmd("TM2",timeout);

  return median(offset);
}

long long int getScanOffset(hokuyo::Laser &laser, int reps)
{
  std::vector<long long int> offset(reps);
  int timeout = 1000;

  if (reps > 99) 
    reps = 99;
  
  if (laser.requestScans(1, -2, 2, 0, 0, reps, timeout) != 0)
  {
    fprintf(stderr, "Error requesting scan.\n");
    return 1;
  }

  hokuyo::LaserScan scan;
  for (int i = 0; i < reps; i++)
  {
    laser.serviceScan(scan, timeout);
    offset[i] = scan.self_time_stamp - scan.system_time_stamp;
    printf("%lli %lli - ", scan.self_time_stamp, scan.system_time_stamp);
  }

  return median(offset);
}

long long int getDataOffset(hokuyo::Laser &laser, int cycles)
{
  int ckreps = 1;
  int scanreps = 1;
  long long int pre = getClockOffset(laser, ckreps);
  std::vector<long long int> samples(cycles);
  for (int i = 0; i < cycles; i++)
  {                                                 
    long long int scan = getScanOffset(laser, ckreps);
    long long int post = getClockOffset(laser, scanreps);
    samples[i] = scan - (post+pre)/2;
//    printf("%lli %lli %lli %lli\n", samples[i], post, pre, scan);
//    fflush(stdout);
    pre = post;
  }

  printf("%lli\n", median(samples));
  return median(samples);
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    fprintf(stderr, "usage: timestamp_test <port>\n");
    return 1;
  }

  char *port = argv[1];

  hokuyo::Laser laser;

  //printf("# %s\n", laser.getFirmwareVersion().c_str());
  //fprintf(stderr, "%s\n", laser.getFirmwareVersion().c_str());

  /*
  int timeout = 1000;
  laser.sendCmd("TM0",timeout);
  long long int pc_start = timeHelper();
  laser.sendCmd("TM1",timeout);
  long long int hokuyo_start = laser.readTime();
  for (int i = 0; i < 10000; i++)
  {
    laser.sendCmd("TM1",timeout);
    long long int hokuyo_time = laser.readTime() - hokuyo_start;
    long long int pc_time = timeHelper() - pc_start;
    printf("%lli %lli\n", hokuyo_time, pc_time);
    fflush(stdout);
  }
  */
  /*int timeout = 1000;
  for (int i = 0; i < 10; i++)
  {
    fprintf(stderr, ".");
    laser.sendCmd("TM0",timeout);
    laser.sendCmd("TM1",timeout);
    long long int hokuyo_time = laser.readTime();
    long long int pc_time = timeHelper();
    laser.sendCmd("TM2",timeout);
   
    if (laser.requestScans(0, -1, 1, 0, 0, 1, timeout) != 0)
    {
      fprintf(stderr, "Error requesting scan.\n");
      return 1;
    }

    hokuyo::LaserScan scan;
    laser.serviceScan(scan, timeout);
    long long int hokuyo_time2 = scan.self_time_stamp;
    long long int pc_time2 = scan.system_time_stamp;

    printf("%lli %lli %lli %lli\n", hokuyo_time, pc_time, hokuyo_time2, pc_time2);
    fflush(stdout);
  }*/

  //long long int initial_offset = getClockOffset(laser, 20);
  //long long int end_offset = getClockOffset(laser, 20);
  
  /*while (true)
  {
    getDataOffset(laser, 1);
    fflush(stdout);
  } */
  
  hokuyo::LaserScan  scan_;

  laser.open(port);
  while (true)
  {
    try {
      //std::string status = laser.getStatus();
      //if (status == std::string("Sensor works well."))
      {
        laser.laserOn(); 
        laser.requestScans(true, -2.3562, 2.3562, 0, 0);
        //laser.serviceScan(scan_);
        laser.serviceScan(scan_);
      }
      //else
      //  fprintf(stderr, "%s\n", laser.getStatus().c_str());
    }
    catch (hokuyo::Exception e)
    {
      fprintf(stderr, "%s\n", e.what());
    }
    try {
        laser.stopScanning();
    }
    catch (hokuyo::Exception e)
    {
      fprintf(stderr, "%s\n", e.what());
    }
    //usleep(100000);
    try {
      //laser.close();
    }
    catch (hokuyo::Exception e)
    {
      fprintf(stderr, "%s\n", e.what());
    }
  }
}
