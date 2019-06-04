#include <queue>

class CMovingAverage
{
public:
	CMovingAverage(size_t a_nDataPoints) : NDATAPOINTS(a_nDataPoints) {};
	virtual ~CMovingAverage() {};

	virtual void AddSample(const float a_fSample) = 0;
	virtual float GetAverage() const = 0;

protected:
	const size_t NDATAPOINTS;
};

class CWeightedMovingAverage : public CMovingAverage
{
public:
	CWeightedMovingAverage(size_t a_nDataPoints = 10);
	virtual ~CWeightedMovingAverage();

	virtual void AddSample(const float a_fSample);
	virtual float GetAverage() const;

private:	
	float m_fTotal;
	float m_fNumerator;
	std::queue<float> m_qSamples;
};

inline CWeightedMovingAverage::CWeightedMovingAverage(size_t a_nDataPoints)
: CMovingAverage(a_nDataPoints)
, m_fTotal(0)
, m_fNumerator(0)
{
}

inline CWeightedMovingAverage::~CWeightedMovingAverage()
{
	while(!m_qSamples.empty())
	{
		m_qSamples.pop();
	}
}

inline void CWeightedMovingAverage::AddSample(const float a_fSample)
{
	if(m_qSamples.size() >= NDATAPOINTS)
	{
		float fOldestSample = m_qSamples.front();
		m_qSamples.pop();

		m_qSamples.push(a_fSample);
		m_fNumerator += m_qSamples.size()*m_qSamples.back() - m_fTotal;
		m_fTotal     += m_qSamples.back() - fOldestSample;	
	}
	else
	{
		m_qSamples.push(a_fSample);
		m_fNumerator += m_qSamples.size()*m_qSamples.back();
		m_fTotal     += m_qSamples.back();
	}
}

inline float CWeightedMovingAverage::GetAverage() const
{
	return static_cast<float>
		( m_fNumerator/(m_qSamples.size() * (m_qSamples.size() + 1) / 2) );
}

class CSimpleMovingAverage : public CMovingAverage
{
public:
	CSimpleMovingAverage(size_t a_nDataPoints = 10);
	virtual ~CSimpleMovingAverage();

	virtual void AddSample(const float a_fSample);
	virtual float GetAverage() const;

private:
	float m_fAverage;
	std::queue<float> m_qSamples;
};

inline CSimpleMovingAverage::CSimpleMovingAverage(size_t a_nDataPoints)
: CMovingAverage(a_nDataPoints)
, m_fAverage(0)
{
}

inline CSimpleMovingAverage::~CSimpleMovingAverage()
{
	while(!m_qSamples.empty())
	{
		m_qSamples.pop();
	}
}

inline void CSimpleMovingAverage::AddSample(const float a_fSample)
{
	if(m_qSamples.size() >= NDATAPOINTS)
	{
		float fOldestSample = m_qSamples.front();
		m_qSamples.pop();

		m_qSamples.push(a_fSample);
		m_fAverage += (a_fSample - fOldestSample)/NDATAPOINTS;
	}
	else
	{
		float fPrevSum = (m_qSamples.size() * m_fAverage); 
		m_qSamples.push(a_fSample);
		m_fAverage = ( (fPrevSum + a_fSample)/m_qSamples.size() );
	}
}

inline float CSimpleMovingAverage::GetAverage() const
{
	return m_fAverage;
}
