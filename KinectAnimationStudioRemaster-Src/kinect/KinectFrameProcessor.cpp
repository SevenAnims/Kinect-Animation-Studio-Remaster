#include "KinectFrameProcessor.h"

/// <summary>
/// Constructor
/// </summary>
/// <param name="kSensor" >Sensor that frame processor will be attached to</param>
KinectFrameProcessor::KinectFrameProcessor(IKinectSensor *kSensor) :
m_pKinectSensor(NULL),
m_pBodyFrameReader(NULL),
m_pCoordinateMapper(NULL),
m_pSubscriberListMutex(NULL)
{
	// Alocate new mutex
	m_pSubscriberListMutex = new std::timed_mutex;

	if (kSensor)
		init(kSensor);
}

/// <summary>
/// Destructor
/// </summary>
KinectFrameProcessor::~KinectFrameProcessor() 
{
	stop();

	if (m_pSubscriberListMutex) {
		delete m_pSubscriberListMutex;
		m_pSubscriberListMutex = NULL;
	}
}

void KinectFrameProcessor::stop()
{
	// Ask the loop to exit first; Process() leaves within its next timeout.
	quitMain = true;

	// Join before anything the thread touches is released. When init() failed
	// early the future has no shared state, and wait_for() on it throws
	// std::future_error - inside this path that would terminate the process.
	if (m_pmainThread.valid()) {
		try { m_pmainThread.wait(); }
		catch (...) {}
	}

	// No frame may be delivered once the thread is gone.
	if (m_pBodyFrameReader && m_hFrameEvent) {
		m_pBodyFrameReader->UnsubscribeFrameArrived(m_hFrameEvent);
		m_hFrameEvent = 0;
	}

	unsubscribeAll();
}

/// <summary>
/// Initializes this object by attaching a sensor to it
/// </summary>
/// <param name="kSensor" >Sensor that frame processor will be attached to</param>
HRESULT KinectFrameProcessor::init(IKinectSensor *kSensor) {

	// Re-init must first park the running thread: assigning a new std::async
	// over a future nobody waited on blocks in the old future's destructor.
	if (m_pmainThread.valid())
		stop();

	if (!kSensor)
		return E_POINTER;

	m_pKinectSensor = kSensor;
	RetrieveKinectSensorStructures(m_pKinectSensor, &m_pCoordinateMapper, &m_pBodyFrameReader);

	// Attachment failed
	if ( !(m_pKinectSensor && m_pCoordinateMapper && m_pBodyFrameReader))
		return -1;


	// Subscribes to frame listener
	m_pBodyFrameReader->SubscribeFrameArrived(&m_hFrameEvent);

	// Starts internal frame processing
	quitMain = false;
	
	auto processing_function_def = std::bind(&KinectFrameProcessor::Process, this);
	m_pmainThread = std::async(std::launch::async, 
		[this] {
			Process();
			return true;
		});

	
	return S_OK;
}

/// <summary>
/// Allow other classes to subscribe to our method,
/// so anytime a frame arrives, they are signaled
/// </summary>
void KinectFrameProcessor::subscribe(const KReader_ptr &subscriber)  {
	if (m_pCoordinateMapper)
		subscriber->setCoordinateMapper(m_pCoordinateMapper);
	subscriberList.push_back(subscriber);
}

/// <summary>
/// All subscribers are removed from the list
/// </summary>
void KinectFrameProcessor::unsubscribeAll()  {
	if (!m_pSubscriberListMutex)
		return;
	std::unique_lock<std::timed_mutex> lock_list(*m_pSubscriberListMutex, std::defer_lock);
	// Give up instead of throwing: this runs from the destructor during
	// shutdown, where an exception would terminate the process.
	if (!lock_list.try_lock_for(std::chrono::milliseconds(800)))
		return;

	subscriberList.clear();
}


/// <summary>
/// Main processing thread
/// </summary>
void KinectFrameProcessor::Process() {

	if (!m_pBodyFrameReader)
		return;

	HRESULT hr;

	while (!quitMain) {

		// Wait for event
		WaitForSingleObject(reinterpret_cast<HANDLE>(m_hFrameEvent), 500);

		// Arrived data
		IBodyFrameArrivedEventArgs* pBodyArgs = nullptr;
		hr = m_pBodyFrameReader->GetFrameArrivedEventData(m_hFrameEvent, &pBodyArgs);

		if (SUCCEEDED(hr)) {
			// Frame Reference
			IBodyFrameReference* pBodyReference = nullptr;
			hr = pBodyArgs->get_FrameReference(&pBodyReference);

			if (SUCCEEDED(hr)){
				// Frame
				IBodyFrame* pBodyFrame = nullptr;
				hr = pBodyReference->AcquireFrame(&pBodyFrame);
				
				// Frame time
				INT64 nTime = 0;
				if (SUCCEEDED(hr)){
					hr = pBodyFrame->get_RelativeTime(&nTime);
				}
				if (SUCCEEDED(hr)){
					// Try locking the list, so we can safely notify everyone
					std::unique_lock<std::timed_mutex> lock_list(*m_pSubscriberListMutex, std::defer_lock);
					// Failed to lock, just continue
					if (lock_list.try_lock_for(std::chrono::milliseconds(100))) {
						// If we arrived here, frame has been succesfully acquired
						// Notify subscribers
						for (auto& it : subscriberList) {
							it->notify(pBodyFrame, nTime);
						}
					}

				}
				SafeRelease(pBodyFrame);
			}
			// Both hold a reference on the event data of every single frame;
			// leaking them costs two COM references per frame.
			SafeRelease(pBodyReference);
		}
		SafeRelease(pBodyArgs);
	} // END While
}
