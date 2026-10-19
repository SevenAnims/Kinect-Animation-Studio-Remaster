#include "KDepthReader.h"
#include <cstring>

/// <summary>
/// Constructor
/// </summary>
KDepthReader::KDepthReader()
: m_pSensor(NULL),
  m_pDepthFrameReader(NULL),
  m_hFrameEvent(NULL),
  m_running(false),
  m_hasData(false),
  m_width(0),
  m_height(0)
{
}

/// <summary>
/// Destructor
/// </summary>
KDepthReader::~KDepthReader()
{
	stop();
}

/// <summary>
/// Opens the depth stream reader and starts the reading thread
/// </summary>
HRESULT KDepthReader::init(IKinectSensor *sensor)
{
	stop();

	m_pSensor = sensor;
	if (!sensor)
		return E_INVALIDARG;

	HRESULT hr = sensor->Open();
	if (FAILED(hr))
		return hr;

	IDepthFrameSource *pSource = NULL;
	hr = sensor->get_DepthFrameSource(&pSource);
	if (FAILED(hr))
		return hr;

	hr = pSource->OpenReader(&m_pDepthFrameReader);
	if (FAILED(hr))
	{
		SafeRelease(pSource);
		return hr;
	}

	IFrameDescription *pDesc = NULL;
	hr = pSource->get_FrameDescription(&pDesc);
	if (SUCCEEDED(hr) && pDesc)
	{
		int w = 0, h = 0;
		pDesc->get_Width(&w);
		pDesc->get_Height(&h);
		if (w > 0 && h > 0)
		{
			m_width = UINT(w);
			m_height = UINT(h);
		}
	}
	SafeRelease(pDesc);
	SafeRelease(pSource);

	if (m_width == 0 || m_height == 0)
	{
		m_width = 512;
		m_height = 424;
	}

	m_depth.assign(m_width * m_height, 0);
	m_hasData = false;

	m_pDepthFrameReader->SubscribeFrameArrived(&m_hFrameEvent);

	m_running = true;
	m_thread = std::async(std::launch::async, [this] { Process(); });
	return S_OK;
}

/// <summary>
/// Stops the reading thread and releases the reader
/// </summary>
void KDepthReader::stop()
{
	if (m_running)
	{
		m_running = false;
		if (m_thread.valid())
		{
			try { m_thread.wait(); }
			catch (...) {}
		}
	}

	if (m_pDepthFrameReader)
	{
		if (m_hFrameEvent)
			m_pDepthFrameReader->UnsubscribeFrameArrived(m_hFrameEvent);
		SafeRelease(m_pDepthFrameReader);
	}
	m_hFrameEvent = NULL;
}

/// <summary>
/// Depth reading thread
/// </summary>
void KDepthReader::Process()
{
	HRESULT hrCom = CoInitializeEx(NULL, COINIT_MULTITHREADED);

	while (m_running)
	{
		WaitForSingleObject(reinterpret_cast<HANDLE>(m_hFrameEvent), 200);

		IDepthFrameArrivedEventArgs *pArgs = NULL;
		if (FAILED(m_pDepthFrameReader->GetFrameArrivedEventData(m_hFrameEvent, &pArgs)))
			continue;

		IDepthFrameReference *pRef = NULL;
		if (FAILED(pArgs->get_FrameReference(&pRef)))
		{
			SafeRelease(pArgs);
			continue;
		}

		IDepthFrame *pFrame = NULL;
		if (SUCCEEDED(pRef->AcquireFrame(&pFrame)))
		{
			UINT capacity = 0;
			UINT16 *buffer = NULL;
			if (SUCCEEDED(pFrame->AccessUnderlyingBuffer(&capacity, &buffer)))
			{
				if (buffer && capacity == m_width * m_height)
				{
					std::lock_guard<std::mutex> lock(m_mutex);
					memcpy(m_depth.data(), buffer, m_depth.size() * sizeof(UINT16));
					m_hasData = true;
				}
			}
			SafeRelease(pFrame);
		}

		SafeRelease(pRef);
		SafeRelease(pArgs);
	}

	if (SUCCEEDED(hrCom))
		CoUninitialize();
}

/// <summary>
/// Copies the latest depth frame
/// </summary>
bool KDepthReader::getDepth(UINT &width, UINT &height, std::vector<UINT16> &out)
{
	if (!m_hasData)
		return false;

	std::lock_guard<std::mutex> lock(m_mutex);
	width = m_width;
	height = m_height;
	out = m_depth;
	return true;
}
