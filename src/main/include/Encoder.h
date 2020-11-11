#pragma once
  
 #include <memory>
  
 #include <hal/Types.h>
  
 #include "frc/Counter.h"
 #include "frc/CounterBase.h"
 #include "frc/ErrorBase.h"
 #include "frc/PIDSource.h"
 #include "frc/smartdashboard/Sendable.h"
 #include "frc/smartdashboard/SendableHelper.h"
  
 namespace frc {
  
 class DigitalSource;
 class DigitalGlitchFilter;
 class SendableBuilder;
 class DMA;
 class DMASample;
  
 class Encoder : public ErrorBase,
                 public CounterBase,
                 public PIDSource,
                 public Sendable,
                 public SendableHelper<Encoder> {
   friend class DMA;
   friend class DMASample;
  
  public:
   enum IndexingType {
     kResetWhileHigh,
     kResetWhileLow,
     kResetOnFallingEdge,
     kResetOnRisingEdge
   };
  
   Encoder(int aChannel, int bChannel, bool reverseDirection = false,
           EncodingType encodingType = k4X);
  
   Encoder(DigitalSource* aSource, DigitalSource* bSource,
           bool reverseDirection = false, EncodingType encodingType = k4X);
  
   Encoder(DigitalSource& aSource, DigitalSource& bSource,
           bool reverseDirection = false, EncodingType encodingType = k4X);
  
   Encoder(std::shared_ptr<DigitalSource> aSource,
           std::shared_ptr<DigitalSource> bSource, bool reverseDirection = false,
           EncodingType encodingType = k4X);
  
   ~Encoder() override;
  
   Encoder(Encoder&&) = default;
   Encoder& operator=(Encoder&&) = default;
  
   // CounterBase interface
   int Get() const override;
  
   void Reset() override;
  
   double GetPeriod() const override;
  
   void SetMaxPeriod(double maxPeriod) override;
  
   bool GetStopped() const override;
  
   bool GetDirection() const override;
  
   int GetRaw() const;
  
   int GetEncodingScale() const;
  
   double GetDistance() const;
  
   double GetRate() const;
  
   void SetMinRate(double minRate);
  
   void SetDistancePerPulse(double distancePerPulse);
  
   double GetDistancePerPulse() const;
  
   void SetReverseDirection(bool reverseDirection);
  
   void SetSamplesToAverage(int samplesToAverage);
  
   int GetSamplesToAverage() const;
  
   double PIDGet() override;
  
   void SetIndexSource(int channel, IndexingType type = kResetOnRisingEdge);
  
   void SetIndexSource(const DigitalSource& source,
                       IndexingType type = kResetOnRisingEdge);
  
   void SetSimDevice(HAL_SimDeviceHandle device);
  
   int GetFPGAIndex() const;
  
   void InitSendable(SendableBuilder& builder) override;
  
  private:
   void InitEncoder(bool reverseDirection, EncodingType encodingType);
  
   double DecodingScaleFactor() const;
  
   std::shared_ptr<DigitalSource> m_aSource;  // The A phase of the quad encoder
   std::shared_ptr<DigitalSource> m_bSource;  // The B phase of the quad encoder
   std::shared_ptr<DigitalSource> m_indexSource = nullptr;
   hal::Handle<HAL_EncoderHandle> m_encoder;
  
   friend class DigitalGlitchFilter;
 };
  
 }  // namespace frc