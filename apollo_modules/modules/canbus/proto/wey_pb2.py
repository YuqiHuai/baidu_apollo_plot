"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
_sym_db = _symbol_database.Default()
DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x1emodules/canbus/proto/wey.proto\x12\rapollo.canbus"\xdc\x02\n\x0fAds_shifter_115\x12G\n\rads_shiftmode\x18\x01 \x01(\x0e20.apollo.canbus.Ads_shifter_115.Ads_shiftmodeType\x12I\n\x0eads_targetgear\x18\x02 \x01(\x0e21.apollo.canbus.Ads_shifter_115.Ads_targetgearType"G\n\x11Ads_shiftmodeType\x12\x19\n\x15ADS_SHIFTMODE_INVALID\x10\x00\x12\x17\n\x13ADS_SHIFTMODE_VALID\x10\x01"l\n\x12Ads_targetgearType\x12\x14\n\x10ADS_TARGETGEAR_N\x10\x00\x12\x14\n\x10ADS_TARGETGEAR_R\x10\x01\x12\x14\n\x10ADS_TARGETGEAR_P\x10\x02\x12\x14\n\x10ADS_TARGETGEAR_D\x10\x03"\xb1\x01\n\x0bAds_eps_113\x12?\n\x0bads_epsmode\x18\x01 \x01(\x0e2*.apollo.canbus.Ads_eps_113.Ads_epsmodeType\x12\x1d\n\x15ads_reqepstargetangle\x18\x02 \x01(\x01"B\n\x0fAds_epsmodeType\x12\x17\n\x13ADS_EPSMODE_DISABLE\x10\x00\x12\x16\n\x12ADS_EPSMODE_ACTIVE\x10\x02"\x93+\n\nStatus_310\x12J\n\x11longitudeaccvalid\x18\x01 \x01(\x0e2/.apollo.canbus.Status_310.LongitudeaccvalidType\x12H\n\x10lateralaccevalid\x18\x02 \x01(\x0e2..apollo.canbus.Status_310.LateralaccevalidType\x12L\n\x12vehdynyawratevalid\x18\x03 \x01(\x0e20.apollo.canbus.Status_310.VehdynyawratevalidType\x12F\n\x0fflwheelspdvalid\x18\x04 \x01(\x0e2-.apollo.canbus.Status_310.FlwheelspdvalidType\x12F\n\x0ffrwheelspdvalid\x18\x05 \x01(\x0e2-.apollo.canbus.Status_310.FrwheelspdvalidType\x12F\n\x0frlwheelspdvalid\x18\x06 \x01(\x0e2-.apollo.canbus.Status_310.RlwheelspdvalidType\x12F\n\x0frrwheelspdvalid\x18\x07 \x01(\x0e2-.apollo.canbus.Status_310.RrwheelspdvalidType\x12F\n\x0fvehiclespdvalid\x18\x08 \x01(\x0e2-.apollo.canbus.Status_310.VehiclespdvalidType\x12P\n\x14longitudedrivingmode\x18\t \x01(\x0e22.apollo.canbus.Status_310.LongitudedrivingmodeType\x12>\n\x0bengspdvalid\x18\n \x01(\x0e2).apollo.canbus.Status_310.EngspdvalidType\x12J\n\x11accepedaloverride\x18\x0b \x01(\x0e2/.apollo.canbus.Status_310.AccepedaloverrideType\x12H\n\x10brakepedalstatus\x18\x0c \x01(\x0e2..apollo.canbus.Status_310.BrakepedalstatusType\x12H\n\x10espbrakelightsts\x18\r \x01(\x0e2..apollo.canbus.Status_310.EspbrakelightstsType\x12N\n\x13epbswtpositionvalid\x18\x0e \x01(\x0e21.apollo.canbus.Status_310.EpbswtpositionvalidType\x124\n\x06epbsts\x18\x0f \x01(\x0e2$.apollo.canbus.Status_310.EpbstsType\x12H\n\x10currentgearvalid\x18\x10 \x01(\x0e2..apollo.canbus.Status_310.CurrentgearvalidType\x12B\n\repstrqsnsrsts\x18\x11 \x01(\x0e2+.apollo.canbus.Status_310.EpstrqsnsrstsType\x12R\n\x15eps_interferdetdvalid\x18\x12 \x01(\x0e23.apollo.canbus.Status_310.Eps_interferdetdvalidType\x12F\n\x0fepshandsdetnsts\x18\x13 \x01(\x0e2-.apollo.canbus.Status_310.EpshandsdetnstsType\x12R\n\x15eps_handsdetnstsvalid\x18\x14 \x01(\x0e23.apollo.canbus.Status_310.Eps_handsdetnstsvalidType\x12N\n\x13steerwheelanglesign\x18\x15 \x01(\x0e21.apollo.canbus.Status_310.SteerwheelanglesignType\x12J\n\x11steerwheelspdsign\x18\x16 \x01(\x0e2/.apollo.canbus.Status_310.SteerwheelspdsignType\x12B\n\rdriverdoorsts\x18\x17 \x01(\x0e2+.apollo.canbus.Status_310.DriverdoorstsType\x12:\n\trldoorsts\x18\x18 \x01(\x0e2\'.apollo.canbus.Status_310.RldoorstsType\x12H\n\x10passengerdoorsts\x18\x19 \x01(\x0e2..apollo.canbus.Status_310.PassengerdoorstsType\x12:\n\trrdoorsts\x18\x1a \x01(\x0e2\'.apollo.canbus.Status_310.RrdoorstsType\x12D\n\x0efrontfoglmpsts\x18\x1b \x01(\x0e2,.apollo.canbus.Status_310.FrontfoglmpstsType\x12B\n\rrearfoglmpsts\x18\x1c \x01(\x0e2+.apollo.canbus.Status_310.RearfoglmpstsType\x12<\n\nlowbeamsts\x18\x1d \x01(\x0e2(.apollo.canbus.Status_310.LowbeamstsType\x12>\n\x0bhighbeamsts\x18\x1e \x01(\x0e2).apollo.canbus.Status_310.HighbeamstsType\x12F\n\x0fleftturnlampsts\x18\x1f \x01(\x0e2-.apollo.canbus.Status_310.LeftturnlampstsType\x12H\n\x10rightturnlampsts\x18  \x01(\x0e2..apollo.canbus.Status_310.RightturnlampstsType\x12@\n\x0cbcm_availsts\x18! \x01(\x0e2*.apollo.canbus.Status_310.Bcm_availstsType\x12>\n\x0bbrakelmpsts\x18" \x01(\x0e2).apollo.canbus.Status_310.BrakelmpstsType"S\n\x15LongitudeaccvalidType\x12\x1d\n\x19LONGITUDEACCVALID_INVALID\x10\x00\x12\x1b\n\x17LONGITUDEACCVALID_VALID\x10\x01"P\n\x14LateralaccevalidType\x12\x1c\n\x18LATERALACCEVALID_INVALID\x10\x00\x12\x1a\n\x16LATERALACCEVALID_VALID\x10\x01"V\n\x16VehdynyawratevalidType\x12\x1e\n\x1aVEHDYNYAWRATEVALID_INVALID\x10\x00\x12\x1c\n\x18VEHDYNYAWRATEVALID_VALID\x10\x01"M\n\x13FlwheelspdvalidType\x12\x1b\n\x17FLWHEELSPDVALID_INVALID\x10\x00\x12\x19\n\x15FLWHEELSPDVALID_VALID\x10\x01"M\n\x13FrwheelspdvalidType\x12\x1b\n\x17FRWHEELSPDVALID_INVALID\x10\x00\x12\x19\n\x15FRWHEELSPDVALID_VALID\x10\x01"M\n\x13RlwheelspdvalidType\x12\x1b\n\x17RLWHEELSPDVALID_INVALID\x10\x00\x12\x19\n\x15RLWHEELSPDVALID_VALID\x10\x01"M\n\x13RrwheelspdvalidType\x12\x1b\n\x17RRWHEELSPDVALID_INVALID\x10\x00\x12\x19\n\x15RRWHEELSPDVALID_VALID\x10\x01"M\n\x13VehiclespdvalidType\x12\x1b\n\x17VEHICLESPDVALID_INVALID\x10\x00\x12\x19\n\x15VEHICLESPDVALID_VALID\x10\x01"\xca\x01\n\x18LongitudedrivingmodeType\x12#\n\x1fLONGITUDEDRIVINGMODE_MANUALMODE\x10\x00\x12)\n%LONGITUDEDRIVINGMODE_AUTOMATICSTANDBY\x10\x01\x12.\n*LONGITUDEDRIVINGMODE_AUTOMATICACCELERATION\x10\x02\x12.\n*LONGITUDEDRIVINGMODE_AUTOMATICDECELERATION\x10\x03"w\n\x0fEngspdvalidType\x12\x17\n\x13ENGSPDVALID_INVALID\x10\x00\x12\x15\n\x11ENGSPDVALID_VALID\x10\x01\x12\x1a\n\x16ENGSPDVALID_INIT_VALUE\x10\x02\x12\x18\n\x14ENGSPDVALID_RESERVED\x10\x03"[\n\x15AccepedaloverrideType\x12"\n\x1eACCEPEDALOVERRIDE_NOT_OVERRIDE\x10\x00\x12\x1e\n\x1aACCEPEDALOVERRIDE_OVERRIDE\x10\x01"\x92\x01\n\x14BrakepedalstatusType\x12 \n\x1cBRAKEPEDALSTATUS_NOT_PRESSED\x10\x00\x12\x1c\n\x18BRAKEPEDALSTATUS_PRESSED\x10\x01\x12\x1e\n\x1aBRAKEPEDALSTATUS_RESERVED1\x10\x02\x12\x1a\n\x16BRAKEPEDALSTATUS_ERROR\x10\x03"I\n\x14EspbrakelightstsType\x12\x18\n\x14ESPBRAKELIGHTSTS_OFF\x10\x00\x12\x17\n\x13ESPBRAKELIGHTSTS_ON\x10\x01"[\n\x17EpbswtpositionvalidType\x12\x1d\n\x19EPBSWTPOSITIONVALID_VALID\x10\x00\x12!\n\x1dEPBSWTPOSITIONVALID_NOT_VALID\x10\x01"`\n\nEpbstsType\x12\x13\n\x0fEPBSTS_RELEASED\x10\x00\x12\x11\n\rEPBSTS_CLOSED\x10\x01\x12\x16\n\x12EPBSTS_IN_PROGRESS\x10\x02\x12\x12\n\x0eEPBSTS_UNKNOWN\x10\x03"P\n\x14CurrentgearvalidType\x12\x1c\n\x18CURRENTGEARVALID_INVALID\x10\x00\x12\x1a\n\x16CURRENTGEARVALID_VALID\x10\x01"I\n\x11EpstrqsnsrstsType\x12\x18\n\x14EPSTRQSNSRSTS_NORMAL\x10\x00\x12\x1a\n\x16EPSTRQSNSRSTS_ABNORMAL\x10\x01"_\n\x19Eps_interferdetdvalidType\x12!\n\x1dEPS_INTERFERDETDVALID_INVALID\x10\x00\x12\x1f\n\x1bEPS_INTERFERDETDVALID_VALID\x10\x01"g\n\x13EpshandsdetnstsType\x12)\n%EPSHANDSDETNSTS_HANDSOFF_NOT_DETECTED\x10\x00\x12%\n!EPSHANDSDETNSTS_HANDOFFF_DETECTED\x10\x01"_\n\x19Eps_handsdetnstsvalidType\x12!\n\x1dEPS_HANDSDETNSTSVALID_INVALID\x10\x00\x12\x1f\n\x1bEPS_HANDSDETNSTSVALID_VALID\x10\x01"h\n\x17SteerwheelanglesignType\x12%\n!STEERWHEELANGLESIGN_LEFT_POSITIVE\x10\x00\x12&\n"STEERWHEELANGLESIGN_RIGHT_NEGATIVE\x10\x01"b\n\x15SteerwheelspdsignType\x12#\n\x1fSTEERWHEELSPDSIGN_LEFT_POSITIVE\x10\x00\x12$\n STEERWHEELSPDSIGN_RIGHT_NEGATIVE\x10\x01"E\n\x11DriverdoorstsType\x12\x18\n\x14DRIVERDOORSTS_CLOSED\x10\x00\x12\x16\n\x12DRIVERDOORSTS_OPEN\x10\x01"9\n\rRldoorstsType\x12\x14\n\x10RLDOORSTS_CLOSED\x10\x00\x12\x12\n\x0eRLDOORSTS_OPEN\x10\x01"N\n\x14PassengerdoorstsType\x12\x1b\n\x17PASSENGERDOORSTS_CLOSED\x10\x00\x12\x19\n\x15PASSENGERDOORSTS_OPEN\x10\x01"9\n\rRrdoorstsType\x12\x14\n\x10RRDOORSTS_CLOSED\x10\x00\x12\x12\n\x0eRRDOORSTS_OPEN\x10\x01"\x82\x01\n\x12FrontfoglmpstsType\x12\x16\n\x12FRONTFOGLMPSTS_OFF\x10\x00\x12\x15\n\x11FRONTFOGLMPSTS_ON\x10\x01\x12\x1b\n\x17FRONTFOGLMPSTS_RESERVED\x10\x02\x12 \n\x1cFRONTFOGLMPSTS_NOT_AVAILABLE\x10\x03"@\n\x11RearfoglmpstsType\x12\x15\n\x11REARFOGLMPSTS_OFF\x10\x00\x12\x14\n\x10REARFOGLMPSTS_ON\x10\x01"7\n\x0eLowbeamstsType\x12\x12\n\x0eLOWBEAMSTS_OFF\x10\x00\x12\x11\n\rLOWBEAMSTS_ON\x10\x01":\n\x0fHighbeamstsType\x12\x13\n\x0fHIGHBEAMSTS_OFF\x10\x00\x12\x12\n\x0eHIGHBEAMSTS_ON\x10\x01"F\n\x13LeftturnlampstsType\x12\x17\n\x13LEFTTURNLAMPSTS_OFF\x10\x00\x12\x16\n\x12LEFTTURNLAMPSTS_ON\x10\x01"I\n\x14RightturnlampstsType\x12\x18\n\x14RIGHTTURNLAMPSTS_OFF\x10\x00\x12\x17\n\x13RIGHTTURNLAMPSTS_ON\x10\x01"\x8a\x01\n\x10Bcm_availstsType\x12\x1c\n\x18BCM_AVAILSTS_MANUAL_MODE\x10\x00\x12 \n\x1cBCM_AVAILSTS_AUTONOMOUS_MODE\x10\x01\x12\x1a\n\x16BCM_AVAILSTS_RESERVED1\x10\x02\x12\x1a\n\x16BCM_AVAILSTS_RESERVED2\x10\x03":\n\x0fBrakelmpstsType\x12\x13\n\x0fBRAKELMPSTS_OFF\x10\x00\x12\x12\n\x0eBRAKELMPSTS_ON\x10\x01"\x1e\n\rVin_resp3_393\x12\r\n\x05vin16\x18\x01 \x01(\x05"\x87\x01\n\rVin_resp2_392\x12\r\n\x05vin15\x18\x01 \x01(\x05\x12\r\n\x05vin14\x18\x02 \x01(\x05\x12\r\n\x05vin13\x18\x03 \x01(\x05\x12\r\n\x05vin12\x18\x04 \x01(\x05\x12\r\n\x05vin11\x18\x05 \x01(\x05\x12\r\n\x05vin10\x18\x06 \x01(\x05\x12\r\n\x05vin09\x18\x07 \x01(\x05\x12\r\n\x05vin08\x18\x08 \x01(\x05"\x87\x01\n\rVin_resp1_391\x12\r\n\x05vin07\x18\x01 \x01(\x05\x12\r\n\x05vin06\x18\x02 \x01(\x05\x12\r\n\x05vin05\x18\x03 \x01(\x05\x12\r\n\x05vin04\x18\x04 \x01(\x05\x12\r\n\x05vin03\x18\x05 \x01(\x05\x12\r\n\x05vin02\x18\x06 \x01(\x05\x12\r\n\x05vin00\x18\x07 \x01(\x05\x12\r\n\x05vin01\x18\x08 \x01(\x05"\xad\x01\n\x0fAds_req_vin_390\x12I\n\x0ereq_vin_signal\x18\x01 \x01(\x0e21.apollo.canbus.Ads_req_vin_390.Req_vin_signalType"O\n\x12Req_vin_signalType\x12\x1d\n\x19REQ_VIN_SIGNAL_NO_REQUEST\x10\x00\x12\x1a\n\x16REQ_VIN_SIGNAL_REQUEST\x10\x01"\x8d\x05\n\x08Ads1_111\x12@\n\rads_dectostop\x18\x01 \x01(\x0e2).apollo.canbus.Ads1_111.Ads_dectostopType\x126\n\x08ads_mode\x18\x02 \x01(\x0e2$.apollo.canbus.Ads1_111.Ads_modeType\x12\x13\n\x0bads_taracce\x18\x03 \x01(\x01\x12F\n\x10ads_driveoff_req\x18\x04 \x01(\x0e2,.apollo.canbus.Ads1_111.Ads_driveoff_reqType\x12\x17\n\x0fads_aeb_taracce\x18\x05 \x01(\x01\x12N\n\x14ads_aeb_tgtdecel_req\x18\x06 \x01(\x0e20.apollo.canbus.Ads1_111.Ads_aeb_tgtdecel_reqType"J\n\x11Ads_dectostopType\x12\x1b\n\x17ADS_DECTOSTOP_NO_DEMAND\x10\x00\x12\x18\n\x14ADS_DECTOSTOP_DEMAND\x10\x01"?\n\x0cAds_modeType\x12\x15\n\x11ADS_MODE_OFF_MODE\x10\x00\x12\x18\n\x14ADS_MODE_ACTIVE_MODE\x10\x03"S\n\x14Ads_driveoff_reqType\x12\x1e\n\x1aADS_DRIVEOFF_REQ_NO_DEMAND\x10\x00\x12\x1b\n\x17ADS_DRIVEOFF_REQ_DEMAND\x10\x01"_\n\x18Ads_aeb_tgtdecel_reqType\x12"\n\x1eADS_AEB_TGTDECEL_REQ_NO_DEMAND\x10\x00\x12\x1f\n\x1bADS_AEB_TGTDECEL_REQ_DEMAND\x10\x01"\x82\x06\n\x08Fbs2_240\x12F\n\x10flwheeldirection\x18\x01 \x01(\x0e2,.apollo.canbus.Fbs2_240.FlwheeldirectionType\x12\x12\n\nfrwheelspd\x18\x02 \x01(\x01\x12P\n\x15rlwheeldrivedirection\x18\x03 \x01(\x0e21.apollo.canbus.Fbs2_240.RlwheeldrivedirectionType\x12\x12\n\nrlwheelspd\x18\x04 \x01(\x01\x12F\n\x10rrwheeldirection\x18\x05 \x01(\x0e2,.apollo.canbus.Fbs2_240.RrwheeldirectionType\x12\x12\n\nrrwheelspd\x18\x06 \x01(\x01\x12\x12\n\nvehiclespd\x18\x07 \x01(\x01"\x8c\x01\n\x14FlwheeldirectionType\x12\x1c\n\x18FLWHEELDIRECTION_INVALID\x10\x00\x12\x1c\n\x18FLWHEELDIRECTION_FORWARD\x10\x01\x12\x1d\n\x19FLWHEELDIRECTION_BACKWARD\x10\x02\x12\x19\n\x15FLWHEELDIRECTION_STOP\x10\x03"\xa5\x01\n\x19RlwheeldrivedirectionType\x12!\n\x1dRLWHEELDRIVEDIRECTION_INVALID\x10\x00\x12!\n\x1dRLWHEELDRIVEDIRECTION_FORWARD\x10\x01\x12"\n\x1eRLWHEELDRIVEDIRECTION_BACKWARD\x10\x02\x12\x1e\n\x1aRLWHEELDRIVEDIRECTION_STOP\x10\x03"\x8c\x01\n\x14RrwheeldirectionType\x12\x1c\n\x18RRWHEELDIRECTION_INVALID\x10\x00\x12\x1c\n\x18RRWHEELDIRECTION_FORWARD\x10\x01\x12\x1d\n\x19RRWHEELDIRECTION_BACKWARD\x10\x02\x12\x19\n\x15RRWHEELDIRECTION_STOP\x10\x03"\xb8\x02\n\x08Fbs1_243\x12\x15\n\rlongitudeacce\x18\x01 \x01(\x01\x12\x13\n\x0blateralacce\x18\x02 \x01(\x01\x12\x15\n\rvehdynyawrate\x18\x03 \x01(\x01\x12\x12\n\nflwheelspd\x18\x04 \x01(\x01\x12F\n\x10frwheeldirection\x18\x05 \x01(\x0e2,.apollo.canbus.Fbs1_243.FrwheeldirectionType"\x8c\x01\n\x14FrwheeldirectionType\x12\x1c\n\x18FRWHEELDIRECTION_INVALID\x10\x00\x12\x1c\n\x18FRWHEELDIRECTION_FORWARD\x10\x01\x12\x1d\n\x19FRWHEELDIRECTION_BACKWARD\x10\x02\x12\x19\n\x15FRWHEELDIRECTION_STOP\x10\x03":\n\x08Fbs4_235\x12\x17\n\x0fsteerwheelangle\x18\x01 \x01(\x01\x12\x15\n\rsteerwheelspd\x18\x02 \x01(\x01"\xa1\x06\n\x08Fail_241\x124\n\x07engfail\x18\x01 \x01(\x0e2#.apollo.canbus.Fail_241.EngfailType\x124\n\x07espfail\x18\x02 \x01(\x0e2#.apollo.canbus.Fail_241.EspfailType\x124\n\x07epbfail\x18\x03 \x01(\x0e2#.apollo.canbus.Fail_241.EpbfailType\x128\n\tshiftfail\x18\x04 \x01(\x0e2%.apollo.canbus.Fail_241.ShiftfailType\x124\n\x07epsfail\x18\x05 \x01(\x0e2#.apollo.canbus.Fail_241.EpsfailType"4\n\x0bEngfailType\x12\x13\n\x0fENGFAIL_NO_FAIL\x10\x00\x12\x10\n\x0cENGFAIL_FAIL\x10\x01":\n\x0bEspfailType\x12\x16\n\x12ESPFAIL_NO_FAILURE\x10\x00\x12\x13\n\x0fESPFAIL_FAILURE\x10\x01"d\n\x0bEpbfailType\x12\x15\n\x11EPBFAIL_UNDEFINED\x10\x00\x12\x14\n\x10EPBFAIL_NO_ERROR\x10\x01\x12\x11\n\rEPBFAIL_ERROR\x10\x02\x12\x15\n\x11EPBFAIL_DIAGNOSIS\x10\x03"\xf2\x01\n\rShiftfailType\x12\x15\n\x11SHIFTFAIL_NO_FAIL\x10\x00\x12&\n"SHIFTFAIL_TRANSMISSION_MALFUNCTION\x10\x01\x12-\n)SHIFTFAIL_TRANSMISSION_P_ENGAGEMENT_FAULT\x10\x02\x120\n,SHIFTFAIL_TRANSMISSION_P_DISENGAGEMENT_FAULT\x10\x03\x12\x16\n\x12SHIFTFAIL_RESERVED\x10\x04\x12)\n%SHIFTFAIL_TRANSMISSION_LIMIT_FUNCTION\x10\x0f"6\n\x0bEpsfailType\x12\x14\n\x10EPSFAIL_NO_FAULT\x10\x00\x12\x11\n\rEPSFAIL_FAULT\x10\x01"\x9e\t\n\x08Fbs3_237\x12\x0e\n\x06engspd\x18\x01 \x01(\x01\x12\x13\n\x0baccpedalpos\x18\x02 \x01(\x01\x12H\n\x11epbswtichposition\x18\x03 \x01(\x0e2-.apollo.canbus.Fbs3_237.EpbswtichpositionType\x12<\n\x0bcurrentgear\x18\x04 \x01(\x0e2\'.apollo.canbus.Fbs3_237.CurrentgearType\x12F\n\x10eps_streeingmode\x18\x05 \x01(\x0e2,.apollo.canbus.Fbs3_237.Eps_streeingmodeType\x12\x1b\n\x13epsdrvinputtrqvalue\x18\x06 \x01(\x01\x12\x1c\n\x14epsconsumedcurrvalue\x18\x07 \x01(\x01\x12:\n\nepscurrmod\x18\x08 \x01(\x0e2&.apollo.canbus.Fbs3_237.EpscurrmodType"\x93\x01\n\x15EpbswtichpositionType\x12\x1d\n\x19EPBSWTICHPOSITION_NEUTRAL\x10\x00\x12\x1d\n\x19EPBSWTICHPOSITION_RELEASE\x10\x01\x12\x1b\n\x17EPBSWTICHPOSITION_APPLY\x10\x02\x12\x1f\n\x1bEPBSWTICHPOSITION_RESERVED1\x10\x03"]\n\x0fCurrentgearType\x12\x11\n\rCURRENTGEAR_P\x10\x00\x12\x11\n\rCURRENTGEAR_R\x10\x01\x12\x11\n\rCURRENTGEAR_N\x10\x02\x12\x11\n\rCURRENTGEAR_D\x10\x03"\xca\x02\n\x14Eps_streeingmodeType\x12\x1b\n\x17EPS_STREEINGMODE_MANUAL\x10\x00\x12$\n EPS_STREEINGMODE_AUTOMATIC_AVAIL\x10\x01\x12/\n+EPS_STREEINGMODE_MANUAL_FROM_DRVNTERFERENCE\x10\x02\x124\n0EPS_STREEINGMODE_MANUAL_FROM_EPS_FAILED_DETECTED\x10\x03\x12(\n$EPS_STREEINGMODE_TEMPORARY_INHIBITED\x10\x04\x12\x1e\n\x1aEPS_STREEINGMODE_RESERVED1\x10\x05\x12\x1e\n\x1aEPS_STREEINGMODE_RESERVED2\x10\x06\x12\x1e\n\x1aEPS_STREEINGMODE_RESERVED3\x10\x07"\xe3\x01\n\x0eEpscurrmodType\x12\x1a\n\x16EPSCURRMOD_NORMAL_MODE\x10\x00\x12\x19\n\x15EPSCURRMOD_SPORT_MODE\x10\x01\x12\x1b\n\x17EPSCURRMOD_COMFORT_MODE\x10\x02\x12\'\n#EPSCURRMOD_MODESELECTIONNOTPOSSIBLE\x10\x03\x12\x19\n\x15EPSCURRMOD_NO_DISPLAY\x10\x04\x12\x1f\n\x1bEPSCURRMOD_CONDITIONNOTMEET\x10\x05\x12\x18\n\x14EPSCURRMOD_RESERVED1\x10\x06"\xbc\x0f\n\x08Ads3_38e\x12D\n\x0fads_bcm_worksts\x18\x01 \x01(\x0e2+.apollo.canbus.Ads3_38e.Ads_bcm_workstsType\x12L\n\x13ads_bcmworkstsvalid\x18\x02 \x01(\x0e2/.apollo.canbus.Ads3_38e.Ads_bcmworkstsvalidType\x12H\n\x11ads_reqcontrolbcm\x18\x03 \x01(\x0e2-.apollo.canbus.Ads3_38e.Ads_reqcontrolbcmType\x12<\n\x0bhighbeamton\x18\x04 \x01(\x0e2\'.apollo.canbus.Ads3_38e.HighbeamtonType\x12>\n\x0cdippedbeamon\x18\x05 \x01(\x0e2(.apollo.canbus.Ads3_38e.DippedbeamonType\x12>\n\x0cturnllighton\x18\x06 \x01(\x0e2(.apollo.canbus.Ads3_38e.TurnllightonType\x12F\n\x10emergencylighton\x18\x07 \x01(\x0e2,.apollo.canbus.Ads3_38e.EmergencylightonType\x12:\n\nffoglampon\x18\x08 \x01(\x0e2&.apollo.canbus.Ads3_38e.FfoglamponType\x12:\n\nrfoglampon\x18\t \x01(\x0e2&.apollo.canbus.Ads3_38e.RfoglamponType\x12:\n\nbrakelight\x18\n \x01(\x0e2&.apollo.canbus.Ads3_38e.BrakelightType\x122\n\x06hornon\x18\x0b \x01(\x0e2".apollo.canbus.Ads3_38e.HornonType\x12F\n\x10fwindshieldwiper\x18\x0c \x01(\x0e2,.apollo.canbus.Ads3_38e.FwindshieldwiperType\x12F\n\x10rwindshieldwiper\x18\r \x01(\x0e2,.apollo.canbus.Ads3_38e.RwindshieldwiperType"\x86\x01\n\x13Ads_bcm_workstsType\x12\x1b\n\x17ADS_BCM_WORKSTS_DISABLE\x10\x00\x12\x1a\n\x16ADS_BCM_WORKSTS_ENABLE\x10\x01\x12\x1a\n\x16ADS_BCM_WORKSTS_ACTIVE\x10\x02\x12\x1a\n\x16ADS_BCM_WORKSTS_FAILED\x10\x03"Y\n\x17Ads_bcmworkstsvalidType\x12\x1f\n\x1bADS_BCMWORKSTSVALID_INVALID\x10\x00\x12\x1d\n\x19ADS_BCMWORKSTSVALID_VALID\x10\x01"X\n\x15Ads_reqcontrolbcmType\x12 \n\x1cADS_REQCONTROLBCM_NO_REQUEST\x10\x00\x12\x1d\n\x19ADS_REQCONTROLBCM_REQUEST\x10\x01"D\n\x0fHighbeamtonType\x12\x18\n\x14HIGHBEAMTON_TURN_OFF\x10\x00\x12\x17\n\x13HIGHBEAMTON_TURN_ON\x10\x01"G\n\x10DippedbeamonType\x12\x19\n\x15DIPPEDBEAMON_TURN_OFF\x10\x00\x12\x18\n\x14DIPPEDBEAMON_TURN_ON\x10\x01"\x87\x01\n\x10TurnllightonType\x12\x19\n\x15TURNLLIGHTON_TURN_OFF\x10\x00\x12\x1d\n\x19TURNLLIGHTON_TURN_LEFT_ON\x10\x01\x12\x1e\n\x1aTURNLLIGHTON_TURN_RIGHT_ON\x10\x02\x12\x19\n\x15TURNLLIGHTON_RESERVED\x10\x03"S\n\x14EmergencylightonType\x12\x1d\n\x19EMERGENCYLIGHTON_TURN_OFF\x10\x00\x12\x1c\n\x18EMERGENCYLIGHTON_TURN_ON\x10\x01"A\n\x0eFfoglamponType\x12\x17\n\x13FFOGLAMPON_TURN_OFF\x10\x00\x12\x16\n\x12FFOGLAMPON_TURN_ON\x10\x01"A\n\x0eRfoglamponType\x12\x17\n\x13RFOGLAMPON_TURN_OFF\x10\x00\x12\x16\n\x12RFOGLAMPON_TURN_ON\x10\x01"A\n\x0eBrakelightType\x12\x17\n\x13BRAKELIGHT_TURN_OFF\x10\x00\x12\x16\n\x12BRAKELIGHT_TURN_ON\x10\x01"5\n\nHornonType\x12\x13\n\x0fHORNON_TURN_OFF\x10\x00\x12\x12\n\x0eHORNON_TURN_ON\x10\x01"S\n\x14FwindshieldwiperType\x12\x1d\n\x19FWINDSHIELDWIPER_TURN_OFF\x10\x00\x12\x1c\n\x18FWINDSHIELDWIPER_TURN_ON\x10\x01"S\n\x14RwindshieldwiperType\x12\x1d\n\x19RWINDSHIELDWIPER_TURN_OFF\x10\x00\x12\x1c\n\x18RWINDSHIELDWIPER_TURN_ON\x10\x01"\xa3\x05\n\x03Wey\x127\n\x0fads_shifter_115\x18\x01 \x01(\x0b2\x1e.apollo.canbus.Ads_shifter_115\x12/\n\x0bads_eps_113\x18\x02 \x01(\x0b2\x1a.apollo.canbus.Ads_eps_113\x12-\n\nstatus_310\x18\x03 \x01(\x0b2\x19.apollo.canbus.Status_310\x123\n\rvin_resp3_393\x18\x04 \x01(\x0b2\x1c.apollo.canbus.Vin_resp3_393\x123\n\rvin_resp2_392\x18\x05 \x01(\x0b2\x1c.apollo.canbus.Vin_resp2_392\x123\n\rvin_resp1_391\x18\x06 \x01(\x0b2\x1c.apollo.canbus.Vin_resp1_391\x127\n\x0fads_req_vin_390\x18\x07 \x01(\x0b2\x1e.apollo.canbus.Ads_req_vin_390\x12)\n\x08ads1_111\x18\x08 \x01(\x0b2\x17.apollo.canbus.Ads1_111\x12)\n\x08fbs2_240\x18\t \x01(\x0b2\x17.apollo.canbus.Fbs2_240\x12)\n\x08fbs1_243\x18\n \x01(\x0b2\x17.apollo.canbus.Fbs1_243\x12)\n\x08fbs4_235\x18\x0b \x01(\x0b2\x17.apollo.canbus.Fbs4_235\x12)\n\x08fail_241\x18\x0c \x01(\x0b2\x17.apollo.canbus.Fail_241\x12)\n\x08fbs3_237\x18\r \x01(\x0b2\x17.apollo.canbus.Fbs3_237\x12)\n\x08ads3_38e\x18\x0e \x01(\x0b2\x17.apollo.canbus.Ads3_38e')
_ADS_SHIFTER_115 = DESCRIPTOR.message_types_by_name['Ads_shifter_115']
_ADS_EPS_113 = DESCRIPTOR.message_types_by_name['Ads_eps_113']
_STATUS_310 = DESCRIPTOR.message_types_by_name['Status_310']
_VIN_RESP3_393 = DESCRIPTOR.message_types_by_name['Vin_resp3_393']
_VIN_RESP2_392 = DESCRIPTOR.message_types_by_name['Vin_resp2_392']
_VIN_RESP1_391 = DESCRIPTOR.message_types_by_name['Vin_resp1_391']
_ADS_REQ_VIN_390 = DESCRIPTOR.message_types_by_name['Ads_req_vin_390']
_ADS1_111 = DESCRIPTOR.message_types_by_name['Ads1_111']
_FBS2_240 = DESCRIPTOR.message_types_by_name['Fbs2_240']
_FBS1_243 = DESCRIPTOR.message_types_by_name['Fbs1_243']
_FBS4_235 = DESCRIPTOR.message_types_by_name['Fbs4_235']
_FAIL_241 = DESCRIPTOR.message_types_by_name['Fail_241']
_FBS3_237 = DESCRIPTOR.message_types_by_name['Fbs3_237']
_ADS3_38E = DESCRIPTOR.message_types_by_name['Ads3_38e']
_WEY = DESCRIPTOR.message_types_by_name['Wey']
_ADS_SHIFTER_115_ADS_SHIFTMODETYPE = _ADS_SHIFTER_115.enum_types_by_name['Ads_shiftmodeType']
_ADS_SHIFTER_115_ADS_TARGETGEARTYPE = _ADS_SHIFTER_115.enum_types_by_name['Ads_targetgearType']
_ADS_EPS_113_ADS_EPSMODETYPE = _ADS_EPS_113.enum_types_by_name['Ads_epsmodeType']
_STATUS_310_LONGITUDEACCVALIDTYPE = _STATUS_310.enum_types_by_name['LongitudeaccvalidType']
_STATUS_310_LATERALACCEVALIDTYPE = _STATUS_310.enum_types_by_name['LateralaccevalidType']
_STATUS_310_VEHDYNYAWRATEVALIDTYPE = _STATUS_310.enum_types_by_name['VehdynyawratevalidType']
_STATUS_310_FLWHEELSPDVALIDTYPE = _STATUS_310.enum_types_by_name['FlwheelspdvalidType']
_STATUS_310_FRWHEELSPDVALIDTYPE = _STATUS_310.enum_types_by_name['FrwheelspdvalidType']
_STATUS_310_RLWHEELSPDVALIDTYPE = _STATUS_310.enum_types_by_name['RlwheelspdvalidType']
_STATUS_310_RRWHEELSPDVALIDTYPE = _STATUS_310.enum_types_by_name['RrwheelspdvalidType']
_STATUS_310_VEHICLESPDVALIDTYPE = _STATUS_310.enum_types_by_name['VehiclespdvalidType']
_STATUS_310_LONGITUDEDRIVINGMODETYPE = _STATUS_310.enum_types_by_name['LongitudedrivingmodeType']
_STATUS_310_ENGSPDVALIDTYPE = _STATUS_310.enum_types_by_name['EngspdvalidType']
_STATUS_310_ACCEPEDALOVERRIDETYPE = _STATUS_310.enum_types_by_name['AccepedaloverrideType']
_STATUS_310_BRAKEPEDALSTATUSTYPE = _STATUS_310.enum_types_by_name['BrakepedalstatusType']
_STATUS_310_ESPBRAKELIGHTSTSTYPE = _STATUS_310.enum_types_by_name['EspbrakelightstsType']
_STATUS_310_EPBSWTPOSITIONVALIDTYPE = _STATUS_310.enum_types_by_name['EpbswtpositionvalidType']
_STATUS_310_EPBSTSTYPE = _STATUS_310.enum_types_by_name['EpbstsType']
_STATUS_310_CURRENTGEARVALIDTYPE = _STATUS_310.enum_types_by_name['CurrentgearvalidType']
_STATUS_310_EPSTRQSNSRSTSTYPE = _STATUS_310.enum_types_by_name['EpstrqsnsrstsType']
_STATUS_310_EPS_INTERFERDETDVALIDTYPE = _STATUS_310.enum_types_by_name['Eps_interferdetdvalidType']
_STATUS_310_EPSHANDSDETNSTSTYPE = _STATUS_310.enum_types_by_name['EpshandsdetnstsType']
_STATUS_310_EPS_HANDSDETNSTSVALIDTYPE = _STATUS_310.enum_types_by_name['Eps_handsdetnstsvalidType']
_STATUS_310_STEERWHEELANGLESIGNTYPE = _STATUS_310.enum_types_by_name['SteerwheelanglesignType']
_STATUS_310_STEERWHEELSPDSIGNTYPE = _STATUS_310.enum_types_by_name['SteerwheelspdsignType']
_STATUS_310_DRIVERDOORSTSTYPE = _STATUS_310.enum_types_by_name['DriverdoorstsType']
_STATUS_310_RLDOORSTSTYPE = _STATUS_310.enum_types_by_name['RldoorstsType']
_STATUS_310_PASSENGERDOORSTSTYPE = _STATUS_310.enum_types_by_name['PassengerdoorstsType']
_STATUS_310_RRDOORSTSTYPE = _STATUS_310.enum_types_by_name['RrdoorstsType']
_STATUS_310_FRONTFOGLMPSTSTYPE = _STATUS_310.enum_types_by_name['FrontfoglmpstsType']
_STATUS_310_REARFOGLMPSTSTYPE = _STATUS_310.enum_types_by_name['RearfoglmpstsType']
_STATUS_310_LOWBEAMSTSTYPE = _STATUS_310.enum_types_by_name['LowbeamstsType']
_STATUS_310_HIGHBEAMSTSTYPE = _STATUS_310.enum_types_by_name['HighbeamstsType']
_STATUS_310_LEFTTURNLAMPSTSTYPE = _STATUS_310.enum_types_by_name['LeftturnlampstsType']
_STATUS_310_RIGHTTURNLAMPSTSTYPE = _STATUS_310.enum_types_by_name['RightturnlampstsType']
_STATUS_310_BCM_AVAILSTSTYPE = _STATUS_310.enum_types_by_name['Bcm_availstsType']
_STATUS_310_BRAKELMPSTSTYPE = _STATUS_310.enum_types_by_name['BrakelmpstsType']
_ADS_REQ_VIN_390_REQ_VIN_SIGNALTYPE = _ADS_REQ_VIN_390.enum_types_by_name['Req_vin_signalType']
_ADS1_111_ADS_DECTOSTOPTYPE = _ADS1_111.enum_types_by_name['Ads_dectostopType']
_ADS1_111_ADS_MODETYPE = _ADS1_111.enum_types_by_name['Ads_modeType']
_ADS1_111_ADS_DRIVEOFF_REQTYPE = _ADS1_111.enum_types_by_name['Ads_driveoff_reqType']
_ADS1_111_ADS_AEB_TGTDECEL_REQTYPE = _ADS1_111.enum_types_by_name['Ads_aeb_tgtdecel_reqType']
_FBS2_240_FLWHEELDIRECTIONTYPE = _FBS2_240.enum_types_by_name['FlwheeldirectionType']
_FBS2_240_RLWHEELDRIVEDIRECTIONTYPE = _FBS2_240.enum_types_by_name['RlwheeldrivedirectionType']
_FBS2_240_RRWHEELDIRECTIONTYPE = _FBS2_240.enum_types_by_name['RrwheeldirectionType']
_FBS1_243_FRWHEELDIRECTIONTYPE = _FBS1_243.enum_types_by_name['FrwheeldirectionType']
_FAIL_241_ENGFAILTYPE = _FAIL_241.enum_types_by_name['EngfailType']
_FAIL_241_ESPFAILTYPE = _FAIL_241.enum_types_by_name['EspfailType']
_FAIL_241_EPBFAILTYPE = _FAIL_241.enum_types_by_name['EpbfailType']
_FAIL_241_SHIFTFAILTYPE = _FAIL_241.enum_types_by_name['ShiftfailType']
_FAIL_241_EPSFAILTYPE = _FAIL_241.enum_types_by_name['EpsfailType']
_FBS3_237_EPBSWTICHPOSITIONTYPE = _FBS3_237.enum_types_by_name['EpbswtichpositionType']
_FBS3_237_CURRENTGEARTYPE = _FBS3_237.enum_types_by_name['CurrentgearType']
_FBS3_237_EPS_STREEINGMODETYPE = _FBS3_237.enum_types_by_name['Eps_streeingmodeType']
_FBS3_237_EPSCURRMODTYPE = _FBS3_237.enum_types_by_name['EpscurrmodType']
_ADS3_38E_ADS_BCM_WORKSTSTYPE = _ADS3_38E.enum_types_by_name['Ads_bcm_workstsType']
_ADS3_38E_ADS_BCMWORKSTSVALIDTYPE = _ADS3_38E.enum_types_by_name['Ads_bcmworkstsvalidType']
_ADS3_38E_ADS_REQCONTROLBCMTYPE = _ADS3_38E.enum_types_by_name['Ads_reqcontrolbcmType']
_ADS3_38E_HIGHBEAMTONTYPE = _ADS3_38E.enum_types_by_name['HighbeamtonType']
_ADS3_38E_DIPPEDBEAMONTYPE = _ADS3_38E.enum_types_by_name['DippedbeamonType']
_ADS3_38E_TURNLLIGHTONTYPE = _ADS3_38E.enum_types_by_name['TurnllightonType']
_ADS3_38E_EMERGENCYLIGHTONTYPE = _ADS3_38E.enum_types_by_name['EmergencylightonType']
_ADS3_38E_FFOGLAMPONTYPE = _ADS3_38E.enum_types_by_name['FfoglamponType']
_ADS3_38E_RFOGLAMPONTYPE = _ADS3_38E.enum_types_by_name['RfoglamponType']
_ADS3_38E_BRAKELIGHTTYPE = _ADS3_38E.enum_types_by_name['BrakelightType']
_ADS3_38E_HORNONTYPE = _ADS3_38E.enum_types_by_name['HornonType']
_ADS3_38E_FWINDSHIELDWIPERTYPE = _ADS3_38E.enum_types_by_name['FwindshieldwiperType']
_ADS3_38E_RWINDSHIELDWIPERTYPE = _ADS3_38E.enum_types_by_name['RwindshieldwiperType']
Ads_shifter_115 = _reflection.GeneratedProtocolMessageType('Ads_shifter_115', (_message.Message,), {'DESCRIPTOR': _ADS_SHIFTER_115, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Ads_shifter_115)
Ads_eps_113 = _reflection.GeneratedProtocolMessageType('Ads_eps_113', (_message.Message,), {'DESCRIPTOR': _ADS_EPS_113, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Ads_eps_113)
Status_310 = _reflection.GeneratedProtocolMessageType('Status_310', (_message.Message,), {'DESCRIPTOR': _STATUS_310, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Status_310)
Vin_resp3_393 = _reflection.GeneratedProtocolMessageType('Vin_resp3_393', (_message.Message,), {'DESCRIPTOR': _VIN_RESP3_393, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Vin_resp3_393)
Vin_resp2_392 = _reflection.GeneratedProtocolMessageType('Vin_resp2_392', (_message.Message,), {'DESCRIPTOR': _VIN_RESP2_392, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Vin_resp2_392)
Vin_resp1_391 = _reflection.GeneratedProtocolMessageType('Vin_resp1_391', (_message.Message,), {'DESCRIPTOR': _VIN_RESP1_391, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Vin_resp1_391)
Ads_req_vin_390 = _reflection.GeneratedProtocolMessageType('Ads_req_vin_390', (_message.Message,), {'DESCRIPTOR': _ADS_REQ_VIN_390, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Ads_req_vin_390)
Ads1_111 = _reflection.GeneratedProtocolMessageType('Ads1_111', (_message.Message,), {'DESCRIPTOR': _ADS1_111, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Ads1_111)
Fbs2_240 = _reflection.GeneratedProtocolMessageType('Fbs2_240', (_message.Message,), {'DESCRIPTOR': _FBS2_240, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Fbs2_240)
Fbs1_243 = _reflection.GeneratedProtocolMessageType('Fbs1_243', (_message.Message,), {'DESCRIPTOR': _FBS1_243, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Fbs1_243)
Fbs4_235 = _reflection.GeneratedProtocolMessageType('Fbs4_235', (_message.Message,), {'DESCRIPTOR': _FBS4_235, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Fbs4_235)
Fail_241 = _reflection.GeneratedProtocolMessageType('Fail_241', (_message.Message,), {'DESCRIPTOR': _FAIL_241, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Fail_241)
Fbs3_237 = _reflection.GeneratedProtocolMessageType('Fbs3_237', (_message.Message,), {'DESCRIPTOR': _FBS3_237, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Fbs3_237)
Ads3_38e = _reflection.GeneratedProtocolMessageType('Ads3_38e', (_message.Message,), {'DESCRIPTOR': _ADS3_38E, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Ads3_38e)
Wey = _reflection.GeneratedProtocolMessageType('Wey', (_message.Message,), {'DESCRIPTOR': _WEY, '__module__': 'modules.canbus.proto.wey_pb2'})
_sym_db.RegisterMessage(Wey)
if _descriptor._USE_C_DESCRIPTORS == False:
    DESCRIPTOR._options = None
    _ADS_SHIFTER_115._serialized_start = 50
    _ADS_SHIFTER_115._serialized_end = 398
    _ADS_SHIFTER_115_ADS_SHIFTMODETYPE._serialized_start = 217
    _ADS_SHIFTER_115_ADS_SHIFTMODETYPE._serialized_end = 288
    _ADS_SHIFTER_115_ADS_TARGETGEARTYPE._serialized_start = 290
    _ADS_SHIFTER_115_ADS_TARGETGEARTYPE._serialized_end = 398
    _ADS_EPS_113._serialized_start = 401
    _ADS_EPS_113._serialized_end = 578
    _ADS_EPS_113_ADS_EPSMODETYPE._serialized_start = 512
    _ADS_EPS_113_ADS_EPSMODETYPE._serialized_end = 578
    _STATUS_310._serialized_start = 581
    _STATUS_310._serialized_end = 6104
    _STATUS_310_LONGITUDEACCVALIDTYPE._serialized_start = 3027
    _STATUS_310_LONGITUDEACCVALIDTYPE._serialized_end = 3110
    _STATUS_310_LATERALACCEVALIDTYPE._serialized_start = 3112
    _STATUS_310_LATERALACCEVALIDTYPE._serialized_end = 3192
    _STATUS_310_VEHDYNYAWRATEVALIDTYPE._serialized_start = 3194
    _STATUS_310_VEHDYNYAWRATEVALIDTYPE._serialized_end = 3280
    _STATUS_310_FLWHEELSPDVALIDTYPE._serialized_start = 3282
    _STATUS_310_FLWHEELSPDVALIDTYPE._serialized_end = 3359
    _STATUS_310_FRWHEELSPDVALIDTYPE._serialized_start = 3361
    _STATUS_310_FRWHEELSPDVALIDTYPE._serialized_end = 3438
    _STATUS_310_RLWHEELSPDVALIDTYPE._serialized_start = 3440
    _STATUS_310_RLWHEELSPDVALIDTYPE._serialized_end = 3517
    _STATUS_310_RRWHEELSPDVALIDTYPE._serialized_start = 3519
    _STATUS_310_RRWHEELSPDVALIDTYPE._serialized_end = 3596
    _STATUS_310_VEHICLESPDVALIDTYPE._serialized_start = 3598
    _STATUS_310_VEHICLESPDVALIDTYPE._serialized_end = 3675
    _STATUS_310_LONGITUDEDRIVINGMODETYPE._serialized_start = 3678
    _STATUS_310_LONGITUDEDRIVINGMODETYPE._serialized_end = 3880
    _STATUS_310_ENGSPDVALIDTYPE._serialized_start = 3882
    _STATUS_310_ENGSPDVALIDTYPE._serialized_end = 4001
    _STATUS_310_ACCEPEDALOVERRIDETYPE._serialized_start = 4003
    _STATUS_310_ACCEPEDALOVERRIDETYPE._serialized_end = 4094
    _STATUS_310_BRAKEPEDALSTATUSTYPE._serialized_start = 4097
    _STATUS_310_BRAKEPEDALSTATUSTYPE._serialized_end = 4243
    _STATUS_310_ESPBRAKELIGHTSTSTYPE._serialized_start = 4245
    _STATUS_310_ESPBRAKELIGHTSTSTYPE._serialized_end = 4318
    _STATUS_310_EPBSWTPOSITIONVALIDTYPE._serialized_start = 4320
    _STATUS_310_EPBSWTPOSITIONVALIDTYPE._serialized_end = 4411
    _STATUS_310_EPBSTSTYPE._serialized_start = 4413
    _STATUS_310_EPBSTSTYPE._serialized_end = 4509
    _STATUS_310_CURRENTGEARVALIDTYPE._serialized_start = 4511
    _STATUS_310_CURRENTGEARVALIDTYPE._serialized_end = 4591
    _STATUS_310_EPSTRQSNSRSTSTYPE._serialized_start = 4593
    _STATUS_310_EPSTRQSNSRSTSTYPE._serialized_end = 4666
    _STATUS_310_EPS_INTERFERDETDVALIDTYPE._serialized_start = 4668
    _STATUS_310_EPS_INTERFERDETDVALIDTYPE._serialized_end = 4763
    _STATUS_310_EPSHANDSDETNSTSTYPE._serialized_start = 4765
    _STATUS_310_EPSHANDSDETNSTSTYPE._serialized_end = 4868
    _STATUS_310_EPS_HANDSDETNSTSVALIDTYPE._serialized_start = 4870
    _STATUS_310_EPS_HANDSDETNSTSVALIDTYPE._serialized_end = 4965
    _STATUS_310_STEERWHEELANGLESIGNTYPE._serialized_start = 4967
    _STATUS_310_STEERWHEELANGLESIGNTYPE._serialized_end = 5071
    _STATUS_310_STEERWHEELSPDSIGNTYPE._serialized_start = 5073
    _STATUS_310_STEERWHEELSPDSIGNTYPE._serialized_end = 5171
    _STATUS_310_DRIVERDOORSTSTYPE._serialized_start = 5173
    _STATUS_310_DRIVERDOORSTSTYPE._serialized_end = 5242
    _STATUS_310_RLDOORSTSTYPE._serialized_start = 5244
    _STATUS_310_RLDOORSTSTYPE._serialized_end = 5301
    _STATUS_310_PASSENGERDOORSTSTYPE._serialized_start = 5303
    _STATUS_310_PASSENGERDOORSTSTYPE._serialized_end = 5381
    _STATUS_310_RRDOORSTSTYPE._serialized_start = 5383
    _STATUS_310_RRDOORSTSTYPE._serialized_end = 5440
    _STATUS_310_FRONTFOGLMPSTSTYPE._serialized_start = 5443
    _STATUS_310_FRONTFOGLMPSTSTYPE._serialized_end = 5573
    _STATUS_310_REARFOGLMPSTSTYPE._serialized_start = 5575
    _STATUS_310_REARFOGLMPSTSTYPE._serialized_end = 5639
    _STATUS_310_LOWBEAMSTSTYPE._serialized_start = 5641
    _STATUS_310_LOWBEAMSTSTYPE._serialized_end = 5696
    _STATUS_310_HIGHBEAMSTSTYPE._serialized_start = 5698
    _STATUS_310_HIGHBEAMSTSTYPE._serialized_end = 5756
    _STATUS_310_LEFTTURNLAMPSTSTYPE._serialized_start = 5758
    _STATUS_310_LEFTTURNLAMPSTSTYPE._serialized_end = 5828
    _STATUS_310_RIGHTTURNLAMPSTSTYPE._serialized_start = 5830
    _STATUS_310_RIGHTTURNLAMPSTSTYPE._serialized_end = 5903
    _STATUS_310_BCM_AVAILSTSTYPE._serialized_start = 5906
    _STATUS_310_BCM_AVAILSTSTYPE._serialized_end = 6044
    _STATUS_310_BRAKELMPSTSTYPE._serialized_start = 6046
    _STATUS_310_BRAKELMPSTSTYPE._serialized_end = 6104
    _VIN_RESP3_393._serialized_start = 6106
    _VIN_RESP3_393._serialized_end = 6136
    _VIN_RESP2_392._serialized_start = 6139
    _VIN_RESP2_392._serialized_end = 6274
    _VIN_RESP1_391._serialized_start = 6277
    _VIN_RESP1_391._serialized_end = 6412
    _ADS_REQ_VIN_390._serialized_start = 6415
    _ADS_REQ_VIN_390._serialized_end = 6588
    _ADS_REQ_VIN_390_REQ_VIN_SIGNALTYPE._serialized_start = 6509
    _ADS_REQ_VIN_390_REQ_VIN_SIGNALTYPE._serialized_end = 6588
    _ADS1_111._serialized_start = 6591
    _ADS1_111._serialized_end = 7244
    _ADS1_111_ADS_DECTOSTOPTYPE._serialized_start = 6923
    _ADS1_111_ADS_DECTOSTOPTYPE._serialized_end = 6997
    _ADS1_111_ADS_MODETYPE._serialized_start = 6999
    _ADS1_111_ADS_MODETYPE._serialized_end = 7062
    _ADS1_111_ADS_DRIVEOFF_REQTYPE._serialized_start = 7064
    _ADS1_111_ADS_DRIVEOFF_REQTYPE._serialized_end = 7147
    _ADS1_111_ADS_AEB_TGTDECEL_REQTYPE._serialized_start = 7149
    _ADS1_111_ADS_AEB_TGTDECEL_REQTYPE._serialized_end = 7244
    _FBS2_240._serialized_start = 7247
    _FBS2_240._serialized_end = 8017
    _FBS2_240_FLWHEELDIRECTIONTYPE._serialized_start = 7566
    _FBS2_240_FLWHEELDIRECTIONTYPE._serialized_end = 7706
    _FBS2_240_RLWHEELDRIVEDIRECTIONTYPE._serialized_start = 7709
    _FBS2_240_RLWHEELDRIVEDIRECTIONTYPE._serialized_end = 7874
    _FBS2_240_RRWHEELDIRECTIONTYPE._serialized_start = 7877
    _FBS2_240_RRWHEELDIRECTIONTYPE._serialized_end = 8017
    _FBS1_243._serialized_start = 8020
    _FBS1_243._serialized_end = 8332
    _FBS1_243_FRWHEELDIRECTIONTYPE._serialized_start = 8192
    _FBS1_243_FRWHEELDIRECTIONTYPE._serialized_end = 8332
    _FBS4_235._serialized_start = 8334
    _FBS4_235._serialized_end = 8392
    _FAIL_241._serialized_start = 8395
    _FAIL_241._serialized_end = 9196
    _FAIL_241_ENGFAILTYPE._serialized_start = 8681
    _FAIL_241_ENGFAILTYPE._serialized_end = 8733
    _FAIL_241_ESPFAILTYPE._serialized_start = 8735
    _FAIL_241_ESPFAILTYPE._serialized_end = 8793
    _FAIL_241_EPBFAILTYPE._serialized_start = 8795
    _FAIL_241_EPBFAILTYPE._serialized_end = 8895
    _FAIL_241_SHIFTFAILTYPE._serialized_start = 8898
    _FAIL_241_SHIFTFAILTYPE._serialized_end = 9140
    _FAIL_241_EPSFAILTYPE._serialized_start = 9142
    _FAIL_241_EPSFAILTYPE._serialized_end = 9196
    _FBS3_237._serialized_start = 9199
    _FBS3_237._serialized_end = 10381
    _FBS3_237_EPBSWTICHPOSITIONTYPE._serialized_start = 9576
    _FBS3_237_EPBSWTICHPOSITIONTYPE._serialized_end = 9723
    _FBS3_237_CURRENTGEARTYPE._serialized_start = 9725
    _FBS3_237_CURRENTGEARTYPE._serialized_end = 9818
    _FBS3_237_EPS_STREEINGMODETYPE._serialized_start = 9821
    _FBS3_237_EPS_STREEINGMODETYPE._serialized_end = 10151
    _FBS3_237_EPSCURRMODTYPE._serialized_start = 10154
    _FBS3_237_EPSCURRMODTYPE._serialized_end = 10381
    _ADS3_38E._serialized_start = 10384
    _ADS3_38E._serialized_end = 12364
    _ADS3_38E_ADS_BCM_WORKSTSTYPE._serialized_start = 11257
    _ADS3_38E_ADS_BCM_WORKSTSTYPE._serialized_end = 11391
    _ADS3_38E_ADS_BCMWORKSTSVALIDTYPE._serialized_start = 11393
    _ADS3_38E_ADS_BCMWORKSTSVALIDTYPE._serialized_end = 11482
    _ADS3_38E_ADS_REQCONTROLBCMTYPE._serialized_start = 11484
    _ADS3_38E_ADS_REQCONTROLBCMTYPE._serialized_end = 11572
    _ADS3_38E_HIGHBEAMTONTYPE._serialized_start = 11574
    _ADS3_38E_HIGHBEAMTONTYPE._serialized_end = 11642
    _ADS3_38E_DIPPEDBEAMONTYPE._serialized_start = 11644
    _ADS3_38E_DIPPEDBEAMONTYPE._serialized_end = 11715
    _ADS3_38E_TURNLLIGHTONTYPE._serialized_start = 11718
    _ADS3_38E_TURNLLIGHTONTYPE._serialized_end = 11853
    _ADS3_38E_EMERGENCYLIGHTONTYPE._serialized_start = 11855
    _ADS3_38E_EMERGENCYLIGHTONTYPE._serialized_end = 11938
    _ADS3_38E_FFOGLAMPONTYPE._serialized_start = 11940
    _ADS3_38E_FFOGLAMPONTYPE._serialized_end = 12005
    _ADS3_38E_RFOGLAMPONTYPE._serialized_start = 12007
    _ADS3_38E_RFOGLAMPONTYPE._serialized_end = 12072
    _ADS3_38E_BRAKELIGHTTYPE._serialized_start = 12074
    _ADS3_38E_BRAKELIGHTTYPE._serialized_end = 12139
    _ADS3_38E_HORNONTYPE._serialized_start = 12141
    _ADS3_38E_HORNONTYPE._serialized_end = 12194
    _ADS3_38E_FWINDSHIELDWIPERTYPE._serialized_start = 12196
    _ADS3_38E_FWINDSHIELDWIPERTYPE._serialized_end = 12279
    _ADS3_38E_RWINDSHIELDWIPERTYPE._serialized_start = 12281
    _ADS3_38E_RWINDSHIELDWIPERTYPE._serialized_end = 12364
    _WEY._serialized_start = 12367
    _WEY._serialized_end = 13042