#include <iostream>
#include <chrono>

/*
	CStopWatch class
	- Counts execution time
*/
class CStopWatch
{
public:
	CStopWatch() : m_start(std::chrono::system_clock::now())
	{}

	void Reset()
	{
		m_start = std::chrono::system_clock::now();
	}

	void GetElapsed()
	{
		m_end = std::chrono::system_clock::now();
		// Time in double
		std::chrono::duration<double> elapsed = m_end - m_start;
		std::chrono::nanoseconds nano = m_end - m_start;

		// Default expression
		std::cout << elapsed.count() << " seconds..." << std::endl;
		/*
		// In nano seconds
		std::cout << nano.count() << " nano seconds..." << std::endl;
		// In micro seconds
		std::chrono::microseconds micro = std::chrono::duration_cast<std::chrono::microseconds>(nano);
		std::cout << micro.count() << " micro seconds..." << std::endl;
		// In mili seconds
		std::chrono::milliseconds milli = std::chrono::duration_cast<std::chrono::milliseconds>(nano);
		std::cout << milli.count() << " milli seconds..." << std::endl;
		// In */
		std::chrono::seconds sec = std::chrono::duration_cast<std::chrono::seconds>(nano);
		std::cout << sec.count() << " seconds..." << std::endl;
		
	}

private:
	std::chrono::system_clock::time_point m_start;
	std::chrono::system_clock::time_point m_end;
};
