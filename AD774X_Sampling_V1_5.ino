//----------------------------------------------------------------------
// reading six data registers each sampling period,
// specified by the SamplePeriod variable, and start a new conversion
//----------------------------------------------------------------------
void StartNewConversion(void) {
  TimeTemp = millis();
  AD774X_Write_Single_Register(ADR_CFG, (AD774X_Read_Single_Register(ADR_CFG) & MODES) | SINGLE);
}
void PeriodicSampling(void) {
  if ((millis() - SamplePeriod) > TimeTemp) {  // is the new sample ready?
    AD774X_Read_Registers(ADR_CAP_DATAH, RTxBuff, 6);
    SerialPrintData();
    StartNewConversion();
  }
}
//----------------------------------------------------------------------
//conversion of the obtained data to real values
//----------------------------------------------------------------------
long ConvertCapRawData(void) {
  return long((long)RTxBuff[ADR_CAP_DATAH] << 16) + ((long)RTxBuff[ADR_CAP_DATAM] << 8) + (long)RTxBuff[ADR_CAP_DATAL] - 0x800000;
}
float ConvertCapData(void) {
  long CapacitanceRaw = ConvertCapRawData();
  // different calculation for AD7747 and AD7745/46
  if (AD7747)return (float)CapacitanceRaw / 1024000.0;
  else return (float)CapacitanceRaw / 2048000.0;
}
float ConvertTempData(void) {
  long TemperatureRaw = ((long)RTxBuff[ADR_VT_DATAH] << 16) + ((long)RTxBuff[ADR_VT_DATAM] << 8) + (long)RTxBuff[ADR_VT_DATAL];
  return (float)TemperatureRaw / 2048.0 - 4096.0;
}
void SerialPrintData(void) {
  Capacitance = ConvertCapData();
  Temperature = ConvertTempData();
  // Serial.println("");
  if (Capacitance >= 0)Serial.print(F(" "));
  Serial.print(Capacitance, 6);
  Serial.print(",");
  Serial.println(Temperature, 2);
}
