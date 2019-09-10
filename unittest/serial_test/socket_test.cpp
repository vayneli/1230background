/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "socket.hpp"
using namespace std;

/*! main
 *
 */
int main(int argc, char** argv)
{

//   string s1,s2,s3;
//   s1 = "#C ";
//   s2=" $";
//   s3=s1+" " +to_string(10) + " " +s2;
//   cout << s3 << endl;
//   char data[] = "lol's full name is league of lengend!";
  Socket client;
  client.client_init("192.168.50.162");
  client.send_data(256,365,0);
  cout << "send successful" << endl;
  // Socket server;
  
  // server.server_init();
  // cout << "-------------test successfully !----------------" << endl;
  
  
   //while(1){
    
    // int num = server.receive_data();
    // //if(num == 0)
    //   //break;
    // cout << server.receive_buff << endl;
  //}


  return 0;
}