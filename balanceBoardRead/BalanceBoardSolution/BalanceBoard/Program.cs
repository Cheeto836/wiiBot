using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading;
using WiiDeviceLibrary;

namespace BalanceBoard
{
	class MainClass
	{
		public static void Main (string[] args)
		{
			//connect to the balance board
			IDeviceProvider deviceProvider = DeviceProviderRegistry.CreateSupportedDeviceProvider();
			deviceProvider.DeviceFound += deviceProvider_DeviceFound;
			deviceProvider.DeviceLost += deviceProvider_DeviceLost;
			deviceProvider.StartDiscovering();
		}
		private static void run_cmd(string cmd, string args)
		{
			ProcessStartInfo start = new ProcessStartInfo();
			start.FileName = "/usr/bin/python";
			start.Arguments = string.Format("{0} {1}", cmd, args);
			start.UseShellExecute = false;
			start.RedirectStandardOutput = true;
			start.RedirectStandardInput = true;
			Process process = new Process();
			process.StartInfo = start;
			process.Start();
			using (StreamReader reader = process.StandardOutput)
			{
				using (StreamWriter writer = process.StandardInput)
				{
					process.WaitForInputIdle();
					writer.WriteLine(100);
					string result = reader.ReadToEnd();
					Console.WriteLine(result);
				}
			}
		}

		static void deviceProvider_DeviceLost(object sender, DeviceInfoEventArgs e)
		{
			Console.WriteLine("A device has been lost.");
			IDeviceInfo lostDeviceInfo = e.DeviceInfo;

			if (lostDeviceInfo is IBluetoothDeviceInfo)
			{
				Console.WriteLine("The address of the Bluetooth device is {0}", ((IBluetoothDeviceInfo)lostDeviceInfo).Address);
			}
		}
        static void deviceProvider_DeviceFound(object sender, DeviceInfoEventArgs e)
        {
            IDeviceProvider deviceProvider = (IDeviceProvider)sender;

            IDeviceInfo foundDeviceInfo = e.DeviceInfo;

            IDevice device = deviceProvider.Connect(foundDeviceInfo);
            Console.WriteLine("Connected to the device.");
            
            if (device is IBalanceBoard)
            {
                IBalanceBoard board = (IBalanceBoard)device;
                Console.WriteLine("We have connected to a Balance Board device.");
                // Here we have access to all the operations that we can perform on a Wiimote.
                
                OnBalanceBoardConnected(board);
            }
            
            // If we don't want to be connected to the device, we can disconnect like this:
            device.Disconnect();
        }

		static void OnBalanceBoardConnected(IBalanceBoard board)
		{
			//start python process
			ProcessStartInfo start = new ProcessStartInfo();
			start.FileName = "/usr/bin/python";
			start.Arguments = "/home/cheeto/code/python/test.py";
			start.UseShellExecute =false;
			start.RedirectStandardInput = true;
			Process myProc = new Process();
			myProc.StartInfo = start;
			myProc.Start();
			//board.Updated += board_Update;
			board.SetReportingMode(ReportingMode.Extension);
		    float bottomleft, bottomright, topleft, topright; //board values
			float top, bottom, left, right; //adjusted values
			float throttle, steer;
			board.Initialize();
			while (true)
			{
				//init the board led
				board.Led = false;

				//get the board values in terms of weight (kg)
				bottomleft = board.BottomLeftWeight;
				bottomright = board.BottomRightWeight;
				topleft = board.TopLeftWeight;
				topright = board.TopRightWeight;
				//write vals to file
				/*myProc.StandardInput.WriteLine(bottomleft);
				myProc.StandardInput.WriteLine(bottomright);
				myProc.StandardInput.WriteLine(topleft);
				myProc.StandardInput.WriteLine(topright);*/

				//calculate average for four position
				top = (topleft + topright) / 2;
				bottom = (bottomleft + bottomright) / 2;
				left = (topleft + bottomleft) / 2;
				right = (topright + bottomright) / 2;

				//get motor vals, throttle and steer
				throttle = getPower(top, bottom, 10);
				steer = getPower(left, right, 10);

				Console.Clear();
				Console.Write("bottomleft: ");
				printGraph(bottomleft, 100.0f);
				Console.Write("bottomright: ");
				printGraph(bottomright, 100.0f);
				Console.Write("topright: ");
				printGraph(topright, 100.0f);
				Console.Write("topleft: ");
				printGraph(topleft, 100.0f);
				Console.Write("throtle: ");
				Console.WriteLine(throttle);
				Console.Write("steer: ");
				Console.WriteLine(steer);
				Console.Write("Total weight: ");
				Console.WriteLine(board.TotalWeight);
				Thread.Sleep(50);
			}
		}
		public static float getPower(float top, float bottom, float deadzone)
		{
			const float max = 80.0f;
			float diff = top - bottom; //posative when forward, negative when back
			if (Math.Abs(diff) <= deadzone)
				return 0;
			else 
			{
				float percent = Math.Abs(diff / max);
				float motorVal = 10.0f * percent * Math.Sign(diff);
				return motorVal;
			}
		}

		public static void printGraph(float val, float max)
		{
			float percent = val / max * 100;
			float percentInt = (float)Math.Ceiling(percent);
			for (int i = 0; i < (int)percentInt; i++)
				Console.Write(".");
			Console.WriteLine();
		}

     }
}
