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
  std::string ifname = "eno1";
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
  std::chrono::steady_clock::time_point t0;

  // Init
  t0 = std::chrono::steady_clock::now();
  for (int iter = 1; iter < 500; iter++)
  {
    std::this_thread::sleep_until(t0 + iter * std::chrono::microseconds(4000));

    if (iter == 0)
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

    if (iter == 100)
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

    if (iter == 200)
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

    if (iter == 300)
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

    if (iter == 400)
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

    ec_master.update();
  }

  // Command
  t0 = std::chrono::steady_clock::now();
  for (int iter = 1; iter < 2000; iter++)
  {
    std::this_thread::sleep_until(t0 + iter * std::chrono::microseconds(4000));
    auto t = std::chrono::steady_clock::now() - t0;

    if (iter > 0)
    {
      for (int i = 0; i < n_slaves; i++)
      {
        const uint16 slave_idx = 1 + i;

        // read
        a_pos[i] = ec_master.tx_pdo[slave_idx].position_actual_value;

        // write
        a_pos_cmd[i] = std::sin(M_PI * t.count() / 1000000000.0) * POSITION_STEP_FACTOR;
        ec_master.rx_pdo[slave_idx].target_position = a_pos_cmd[i];

        char record[1024];
        sprintf(record, "%ld\t%d\t%d", t.count(), a_pos[i], a_pos_cmd[i]);
        file << record << std::endl;
      }
    }

    ec_master.update();
  }


  ec_master.close();

  file.close();

  return 0;
}
