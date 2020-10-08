// STL
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <ratio>
//
#include <pthread.h>
#include <unistd.h>
// soem
#include "ethercat.h"

#include "master.h"

#define SAFETY_LIMIT          10000
#define POSITION_STEP_FACTOR  10000


void control_loop(void *arg)
{

}


int main(int argc, char* argv[])
{
  std::string ifname = "enp7s0f1";
  std::vector<std::string> slaves = { "EWDL Servo" };


  esa::ewdl::ethercat::Master ec_master(ifname, slaves);

  if (!ec_master.init())
  {
    return 0;
  }

  // pthread_t pthread;
  // pthread_attr_t pthread_attr;
  // pthread_create()

  const int n_slaves = slaves.size();

  std::vector<int> a_pos; std::vector<int> a_pos_cmd;
  a_pos.resize(n_slaves, 0); a_pos_cmd.resize(n_slaves, 0);


  if (!ec_master.start())
  {
    return 0;
  }


  std::ofstream file("ewdl.log", std::ofstream::out);
  auto prev_time = std::chrono::steady_clock::now();

  int t = -500;
  while (t < 2000)
  {
    std::this_thread::sleep_until(prev_time + std::chrono::microseconds(4000));

    auto curr_time = std::chrono::steady_clock::now();
    auto period = curr_time - prev_time;
    printf("period: %ld us\n", period.count() / 1000);

    prev_time = curr_time;


    if (t == -500)
    {
      std::cout << "Fault Reset ... ";
      if (ec_master.fault_reset())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    if (t == -400)
    {
      std::cout << "Ready to Switch On ... ";
      if (ec_master.ready_to_switch_on())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    if (t == -300)
    {
      std::cout << "Set Zero Position ... ";
      if (ec_master.set_zero_position())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    if (t == -200)
    {
      std::cout << "Switch On ... ";
      if (ec_master.switch_on())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    if (t == -100)
    {
      std::cout << "Start Cyclic Synchronous Position Mode (CSP) ... ";
      if (ec_master.start_cyclic_syncronous_position())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    if (t > 0)
    {
      for (int i = 0; i < n_slaves; i++)
      {
        const uint16 slave_idx = 1 + i;

        // read
        a_pos[i] = ec_master.tx_pdo[slave_idx].position_actual_value;

        // write
        double f = period.count() / 4000000000.0;
        a_pos_cmd[i] = f * std::sin(M_PI/250 * t) * POSITION_STEP_FACTOR;
        ec_master.rx_pdo[slave_idx].target_position = a_pos_cmd[i];
        //printf("target_position: %d\n", a_pos_cmd[i]);
      }
    }

    ec_master.update();
    t++;
  }

  file.close();
  return 0;
}
