#pragma once

#include "..\common\stdafx.h"
#include <Kinect.h>
#include <vector>
#include <mutex>
#include <atomic>
#include <future>

/*
  Reads the Kinect depth stream (512x424, 16-bit millimetres) in the
  background and publishes the latest frame for the 3D viewport.

  The depth sensor works by projecting an infrared pattern, so this stream
  is effectively the "infrared depth" visualization of the scene.
*/
class KDepthReader
{
public:
	KDepthReader();
	~KDepthReader();

	/// <summary>
	/// Opens the depth stream reader on the given sensor
	/// </summary>
	HRESULT init(IKinectSensor *sensor);

	/// <summary>
	/// Stops the reading thread and releases the reader
	/// </summary>
	void stop();

	/// <summary>
	/// True while the reading thread is running
	/// </summary>
	bool isRunning() const { return m_running; }

	/// <summary>
	/// True once at least one depth frame has been received
	/// </summary>
	bool hasData() const { return m_hasData; }

	/// <summary>
	/// Copies the latest depth frame (16-bit millimetres).
	/// Returns false if no frame is available yet.
	/// </summary>
	bool getDepth(UINT &width, UINT &height, std::vector<UINT16> &out);

private:
	void Process();

	IKinectSensor *m_pSensor;
	IDepthFrameReader *m_pDepthFrameReader;
	WAITABLE_HANDLE m_hFrameEvent;

	std::atomic_bool m_running;
	std::atomic_bool m_hasData;

	std::mutex m_mutex;
	UINT m_width;
	UINT m_height;
	std::vector<UINT16> m_depth;

	std::future<void> m_thread;
};
