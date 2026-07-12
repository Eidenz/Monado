//========= Copyright Valve Corporation ============//
// Generated from versioned OpenVR headers.
// Each interface version is copied from the newest header that still contains it.

#pragma once

#include "openvr.h"
#include "openvr_old_types.h"

namespace vr {
static const char* const IVRApplications_001_Version = "IVRApplications_001";

class IVRApplications_001
{
public:
	virtual EVRApplicationError AddApplicationManifest(const char* pchApplicationManifestFullPath, bool bTemporary = false) = 0;
	virtual EVRApplicationError RemoveApplicationManifest(const char* pchApplicationManifestFullPath) = 0;
	virtual bool IsApplicationInstalled(const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationCount() = 0;
	virtual EVRApplicationError GetApplicationKeyByIndex(uint32_t unApplicationIndex, char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError GetApplicationKeyByProcessId(uint32_t unProcessId, char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError LaunchApplication(const char* pchAppKey) = 0;
	virtual EVRApplicationError LaunchDashboardOverlay(const char* pchAppKey) = 0;
	virtual EVRApplicationError IdentifyApplication(uint32_t unProcessId, const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationProcessId(const char* pchAppKey) = 0;
	virtual const char* GetApplicationsErrorNameFromEnum(EVRApplicationError error) = 0;
	virtual uint32_t GetApplicationPropertyString(const char* pchAppKey, EVRApplicationProperty eProperty, char* pchPropertyValueBuffer, uint32_t unPropertyValueBufferLen, EVRApplicationError* peError = nullptr) = 0;
	virtual bool GetApplicationPropertyBool(const char* pchAppKey, EVRApplicationProperty eProperty, EVRApplicationError* peError = nullptr) = 0;
	virtual EVRApplicationError GetHomeApplication(char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError SetHomeApplication(const char* pchAppKey) = 0;
	virtual EVRApplicationError SetApplicationAutoLaunch(const char* pchAppKey, bool bAutoLaunch) = 0;
	virtual bool GetApplicationAutoLaunch(const char* pchAppKey) = 0;
	virtual EVRApplicationError GetStartingApplication(char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationTransitionState GetTransitionState() = 0;
	virtual EVRApplicationError PerformApplicationPrelaunchCheck(const char* pchAppKey) = 0;
	virtual const char* GetApplicationsTransitionStateNameFromEnum(EVRApplicationTransitionState state) = 0;
};


static const char* const IVRApplications_002_Version = "IVRApplications_002";

class IVRApplications_002
{
public:
	virtual EVRApplicationError AddApplicationManifest(const char* pchApplicationManifestFullPath, bool bTemporary = false) = 0;
	virtual EVRApplicationError RemoveApplicationManifest(const char* pchApplicationManifestFullPath) = 0;
	virtual bool IsApplicationInstalled(const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationCount() = 0;
	virtual EVRApplicationError GetApplicationKeyByIndex(uint32_t unApplicationIndex, char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError GetApplicationKeyByProcessId(uint32_t unProcessId, char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError LaunchApplication(const char* pchAppKey) = 0;
	virtual EVRApplicationError LaunchDashboardOverlay(const char* pchAppKey) = 0;
	virtual EVRApplicationError IdentifyApplication(uint32_t unProcessId, const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationProcessId(const char* pchAppKey) = 0;
	virtual const char* GetApplicationsErrorNameFromEnum(EVRApplicationError error) = 0;
	virtual uint32_t GetApplicationPropertyString(const char* pchAppKey, EVRApplicationProperty eProperty, char* pchPropertyValueBuffer, uint32_t unPropertyValueBufferLen, EVRApplicationError* peError = nullptr) = 0;
	virtual bool GetApplicationPropertyBool(const char* pchAppKey, EVRApplicationProperty eProperty, EVRApplicationError* peError = nullptr) = 0;
	virtual EVRApplicationError SetApplicationAutoLaunch(const char* pchAppKey, bool bAutoLaunch) = 0;
	virtual bool GetApplicationAutoLaunch(const char* pchAppKey) = 0;
	virtual EVRApplicationError GetStartingApplication(char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationTransitionState GetTransitionState() = 0;
	virtual EVRApplicationError PerformApplicationPrelaunchCheck(const char* pchAppKey) = 0;
	virtual const char* GetApplicationsTransitionStateNameFromEnum(EVRApplicationTransitionState state) = 0;
	virtual bool IsQuitUserPromptRequested() = 0;
};


static const char* const IVRApplications_003_Version = "IVRApplications_003";

class IVRApplications_003
{
public:
	virtual EVRApplicationError AddApplicationManifest(const char* pchApplicationManifestFullPath, bool bTemporary = false) = 0;
	virtual EVRApplicationError RemoveApplicationManifest(const char* pchApplicationManifestFullPath) = 0;
	virtual bool IsApplicationInstalled(const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationCount() = 0;
	virtual EVRApplicationError GetApplicationKeyByIndex(uint32_t unApplicationIndex, char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError GetApplicationKeyByProcessId(uint32_t unProcessId, char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError LaunchApplication(const char* pchAppKey) = 0;
	virtual EVRApplicationError LaunchDashboardOverlay(const char* pchAppKey) = 0;
	virtual EVRApplicationError IdentifyApplication(uint32_t unProcessId, const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationProcessId(const char* pchAppKey) = 0;
	virtual const char* GetApplicationsErrorNameFromEnum(EVRApplicationError error) = 0;
	virtual uint32_t GetApplicationPropertyString(const char* pchAppKey, EVRApplicationProperty eProperty, char* pchPropertyValueBuffer, uint32_t unPropertyValueBufferLen, EVRApplicationError* peError = nullptr) = 0;
	virtual bool GetApplicationPropertyBool(const char* pchAppKey, EVRApplicationProperty eProperty, EVRApplicationError* peError = nullptr) = 0;
	virtual uint64_t GetApplicationPropertyUint64(const char* pchAppKey, EVRApplicationProperty eProperty, EVRApplicationError* peError = nullptr) = 0;
	virtual EVRApplicationError SetApplicationAutoLaunch(const char* pchAppKey, bool bAutoLaunch) = 0;
	virtual bool GetApplicationAutoLaunch(const char* pchAppKey) = 0;
	virtual EVRApplicationError GetStartingApplication(char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationTransitionState GetTransitionState() = 0;
	virtual EVRApplicationError PerformApplicationPrelaunchCheck(const char* pchAppKey) = 0;
	virtual const char* GetApplicationsTransitionStateNameFromEnum(EVRApplicationTransitionState state) = 0;
	virtual bool IsQuitUserPromptRequested() = 0;
};


static const char* const IVRApplications_004_Version = "IVRApplications_004";

class IVRApplications_004
{
public:
	virtual EVRApplicationError AddApplicationManifest(const char* pchApplicationManifestFullPath, bool bTemporary = false) = 0;
	virtual EVRApplicationError RemoveApplicationManifest(const char* pchApplicationManifestFullPath) = 0;
	virtual bool IsApplicationInstalled(const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationCount() = 0;
	virtual EVRApplicationError GetApplicationKeyByIndex(uint32_t unApplicationIndex, char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError GetApplicationKeyByProcessId(uint32_t unProcessId, char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError LaunchApplication(const char* pchAppKey) = 0;
	virtual EVRApplicationError LaunchDashboardOverlay(const char* pchAppKey) = 0;
	virtual bool CancelApplicationLaunch(const char* pchAppKey) = 0;
	virtual EVRApplicationError IdentifyApplication(uint32_t unProcessId, const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationProcessId(const char* pchAppKey) = 0;
	virtual const char* GetApplicationsErrorNameFromEnum(EVRApplicationError error) = 0;
	virtual uint32_t GetApplicationPropertyString(const char* pchAppKey, EVRApplicationProperty eProperty, char* pchPropertyValueBuffer, uint32_t unPropertyValueBufferLen, EVRApplicationError* peError = nullptr) = 0;
	virtual bool GetApplicationPropertyBool(const char* pchAppKey, EVRApplicationProperty eProperty, EVRApplicationError* peError = nullptr) = 0;
	virtual uint64_t GetApplicationPropertyUint64(const char* pchAppKey, EVRApplicationProperty eProperty, EVRApplicationError* peError = nullptr) = 0;
	virtual EVRApplicationError SetApplicationAutoLaunch(const char* pchAppKey, bool bAutoLaunch) = 0;
	virtual bool GetApplicationAutoLaunch(const char* pchAppKey) = 0;
	virtual EVRApplicationError GetStartingApplication(char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationTransitionState GetTransitionState() = 0;
	virtual EVRApplicationError PerformApplicationPrelaunchCheck(const char* pchAppKey) = 0;
	virtual const char* GetApplicationsTransitionStateNameFromEnum(EVRApplicationTransitionState state) = 0;
	virtual bool IsQuitUserPromptRequested() = 0;
	virtual EVRApplicationError LaunchInternalProcess(const char* pchBinaryPath, const char* pchArguments, const char* pchWorkingDirectory) = 0;
};


static const char* const IVRApplications_005_Version = "IVRApplications_005";

class IVRApplications_005
{
public:
	virtual EVRApplicationError AddApplicationManifest(const char* pchApplicationManifestFullPath, bool bTemporary = false) = 0;
	virtual EVRApplicationError RemoveApplicationManifest(const char* pchApplicationManifestFullPath) = 0;
	virtual bool IsApplicationInstalled(const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationCount() = 0;
	virtual EVRApplicationError GetApplicationKeyByIndex(uint32_t unApplicationIndex, char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError GetApplicationKeyByProcessId(uint32_t unProcessId, char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError LaunchApplication(const char* pchAppKey) = 0;
	virtual EVRApplicationError LaunchTemplateApplication(const char* pchTemplateAppKey, const char* pchNewAppKey, VR_ARRAY_COUNT(unKeys) const AppOverrideKeys_t* pKeys, uint32_t unKeys) = 0;
	virtual EVRApplicationError LaunchDashboardOverlay(const char* pchAppKey) = 0;
	virtual bool CancelApplicationLaunch(const char* pchAppKey) = 0;
	virtual EVRApplicationError IdentifyApplication(uint32_t unProcessId, const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationProcessId(const char* pchAppKey) = 0;
	virtual const char* GetApplicationsErrorNameFromEnum(EVRApplicationError error) = 0;
	virtual uint32_t GetApplicationPropertyString(const char* pchAppKey, EVRApplicationProperty eProperty, char* pchPropertyValueBuffer, uint32_t unPropertyValueBufferLen, EVRApplicationError* peError = nullptr) = 0;
	virtual bool GetApplicationPropertyBool(const char* pchAppKey, EVRApplicationProperty eProperty, EVRApplicationError* peError = nullptr) = 0;
	virtual uint64_t GetApplicationPropertyUint64(const char* pchAppKey, EVRApplicationProperty eProperty, EVRApplicationError* peError = nullptr) = 0;
	virtual EVRApplicationError SetApplicationAutoLaunch(const char* pchAppKey, bool bAutoLaunch) = 0;
	virtual bool GetApplicationAutoLaunch(const char* pchAppKey) = 0;
	virtual EVRApplicationError GetStartingApplication(char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationTransitionState GetTransitionState() = 0;
	virtual EVRApplicationError PerformApplicationPrelaunchCheck(const char* pchAppKey) = 0;
	virtual const char* GetApplicationsTransitionStateNameFromEnum(EVRApplicationTransitionState state) = 0;
	virtual bool IsQuitUserPromptRequested() = 0;
	virtual EVRApplicationError LaunchInternalProcess(const char* pchBinaryPath, const char* pchArguments, const char* pchWorkingDirectory) = 0;
};


static const char* const IVRApplications_006_Version = "IVRApplications_006";

class IVRApplications_006
{
public:
	virtual EVRApplicationError AddApplicationManifest(const char* pchApplicationManifestFullPath, bool bTemporary = false) = 0;
	virtual EVRApplicationError RemoveApplicationManifest(const char* pchApplicationManifestFullPath) = 0;
	virtual bool IsApplicationInstalled(const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationCount() = 0;
	virtual EVRApplicationError GetApplicationKeyByIndex(uint32_t unApplicationIndex, VR_OUT_STRING() char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError GetApplicationKeyByProcessId(uint32_t unProcessId, VR_OUT_STRING() char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError LaunchApplication(const char* pchAppKey) = 0;
	virtual EVRApplicationError LaunchTemplateApplication(const char* pchTemplateAppKey, const char* pchNewAppKey, VR_ARRAY_COUNT(unKeys) const AppOverrideKeys_t* pKeys, uint32_t unKeys) = 0;
	virtual vr::EVRApplicationError LaunchApplicationFromMimeType(const char* pchMimeType, const char* pchArgs) = 0;
	virtual EVRApplicationError LaunchDashboardOverlay(const char* pchAppKey) = 0;
	virtual bool CancelApplicationLaunch(const char* pchAppKey) = 0;
	virtual EVRApplicationError IdentifyApplication(uint32_t unProcessId, const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationProcessId(const char* pchAppKey) = 0;
	virtual const char* GetApplicationsErrorNameFromEnum(EVRApplicationError error) = 0;
	virtual uint32_t GetApplicationPropertyString(const char* pchAppKey, EVRApplicationProperty eProperty, VR_OUT_STRING() char* pchPropertyValueBuffer, uint32_t unPropertyValueBufferLen, EVRApplicationError* peError = nullptr) = 0;
	virtual bool GetApplicationPropertyBool(const char* pchAppKey, EVRApplicationProperty eProperty, EVRApplicationError* peError = nullptr) = 0;
	virtual uint64_t GetApplicationPropertyUint64(const char* pchAppKey, EVRApplicationProperty eProperty, EVRApplicationError* peError = nullptr) = 0;
	virtual EVRApplicationError SetApplicationAutoLaunch(const char* pchAppKey, bool bAutoLaunch) = 0;
	virtual bool GetApplicationAutoLaunch(const char* pchAppKey) = 0;
	virtual EVRApplicationError SetDefaultApplicationForMimeType(const char* pchAppKey, const char* pchMimeType) = 0;
	virtual bool GetDefaultApplicationForMimeType(const char* pchMimeType, VR_OUT_STRING() char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual bool GetApplicationSupportedMimeTypes(const char* pchAppKey, VR_OUT_STRING() char* pchMimeTypesBuffer, uint32_t unMimeTypesBuffer) = 0;
	virtual uint32_t GetApplicationsThatSupportMimeType(const char* pchMimeType, VR_OUT_STRING() char* pchAppKeysThatSupportBuffer, uint32_t unAppKeysThatSupportBuffer) = 0;
	virtual uint32_t GetApplicationLaunchArguments(uint32_t unHandle, VR_OUT_STRING() char* pchArgs, uint32_t unArgs) = 0;
	virtual EVRApplicationError GetStartingApplication(VR_OUT_STRING() char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationTransitionState GetTransitionState() = 0;
	virtual EVRApplicationError PerformApplicationPrelaunchCheck(const char* pchAppKey) = 0;
	virtual const char* GetApplicationsTransitionStateNameFromEnum(EVRApplicationTransitionState state) = 0;
	virtual bool IsQuitUserPromptRequested() = 0;
	virtual EVRApplicationError LaunchInternalProcess(const char* pchBinaryPath, const char* pchArguments, const char* pchWorkingDirectory) = 0;
	virtual uint32_t GetCurrentSceneProcessId() = 0;
};


static const char* const IVRApplications_007_Version = "IVRApplications_007";

class IVRApplications_007
{
public:
	virtual EVRApplicationError AddApplicationManifest(const char* pchApplicationManifestFullPath, bool bTemporary = false) = 0;
	virtual EVRApplicationError RemoveApplicationManifest(const char* pchApplicationManifestFullPath) = 0;
	virtual bool IsApplicationInstalled(const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationCount() = 0;
	virtual EVRApplicationError GetApplicationKeyByIndex(uint32_t unApplicationIndex, VR_OUT_STRING() char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError GetApplicationKeyByProcessId(uint32_t unProcessId, VR_OUT_STRING() char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError LaunchApplication(const char* pchAppKey) = 0;
	virtual EVRApplicationError LaunchTemplateApplication(const char* pchTemplateAppKey, const char* pchNewAppKey, VR_ARRAY_COUNT(unKeys) const AppOverrideKeys_t* pKeys, uint32_t unKeys) = 0;
	virtual vr::EVRApplicationError LaunchApplicationFromMimeType(const char* pchMimeType, const char* pchArgs) = 0;
	virtual EVRApplicationError LaunchDashboardOverlay(const char* pchAppKey) = 0;
	virtual bool CancelApplicationLaunch(const char* pchAppKey) = 0;
	virtual EVRApplicationError IdentifyApplication(uint32_t unProcessId, const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationProcessId(const char* pchAppKey) = 0;
	virtual const char* GetApplicationsErrorNameFromEnum(EVRApplicationError error) = 0;
	virtual uint32_t GetApplicationPropertyString(const char* pchAppKey, EVRApplicationProperty eProperty, VR_OUT_STRING() char* pchPropertyValueBuffer, uint32_t unPropertyValueBufferLen, EVRApplicationError* peError = nullptr) = 0;
	virtual bool GetApplicationPropertyBool(const char* pchAppKey, EVRApplicationProperty eProperty, EVRApplicationError* peError = nullptr) = 0;
	virtual uint64_t GetApplicationPropertyUint64(const char* pchAppKey, EVRApplicationProperty eProperty, EVRApplicationError* peError = nullptr) = 0;
	virtual EVRApplicationError SetApplicationAutoLaunch(const char* pchAppKey, bool bAutoLaunch) = 0;
	virtual bool GetApplicationAutoLaunch(const char* pchAppKey) = 0;
	virtual EVRApplicationError SetDefaultApplicationForMimeType(const char* pchAppKey, const char* pchMimeType) = 0;
	virtual bool GetDefaultApplicationForMimeType(const char* pchMimeType, VR_OUT_STRING() char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual bool GetApplicationSupportedMimeTypes(const char* pchAppKey, VR_OUT_STRING() char* pchMimeTypesBuffer, uint32_t unMimeTypesBuffer) = 0;
	virtual uint32_t GetApplicationsThatSupportMimeType(const char* pchMimeType, VR_OUT_STRING() char* pchAppKeysThatSupportBuffer, uint32_t unAppKeysThatSupportBuffer) = 0;
	virtual uint32_t GetApplicationLaunchArguments(uint32_t unHandle, VR_OUT_STRING() char* pchArgs, uint32_t unArgs) = 0;
	virtual EVRApplicationError GetStartingApplication(VR_OUT_STRING() char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRSceneApplicationState GetSceneApplicationState() = 0;
	virtual EVRApplicationError PerformApplicationPrelaunchCheck(const char* pchAppKey) = 0;
	virtual const char* GetSceneApplicationStateNameFromEnum(EVRSceneApplicationState state) = 0;
	virtual EVRApplicationError LaunchInternalProcess(const char* pchBinaryPath, const char* pchArguments, const char* pchWorkingDirectory) = 0;
	virtual uint32_t GetCurrentSceneProcessId() = 0;
};


static const char* const IVRApplications_008_Version = "IVRApplications_008";

class IVRApplications_008
{
public:
	virtual EVRApplicationError AddApplicationManifest(const char* pchApplicationManifestFullPath, bool bTemporary = false) = 0;
	virtual EVRApplicationError RemoveApplicationManifest(const char* pchApplicationManifestFullPath) = 0;
	virtual bool IsApplicationInstalled(const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationCount() = 0;
	virtual EVRApplicationError GetApplicationKeyByIndex(uint32_t unApplicationIndex, VR_OUT_STRING() char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError GetApplicationKeyByProcessId(uint32_t unProcessId, VR_OUT_STRING() char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRApplicationError LaunchApplication(const char* pchAppKey) = 0;
	virtual EVRApplicationError LaunchTemplateApplication(const char* pchTemplateAppKey, const char* pchNewAppKey, VR_ARRAY_COUNT(unKeys) const AppOverrideKeys_t* pKeys, uint32_t unKeys) = 0;
	virtual vr::EVRApplicationError LaunchApplicationFromMimeType(const char* pchMimeType, const char* pchArgs) = 0;
	virtual EVRApplicationError LaunchDashboardOverlay(const char* pchAppKey) = 0;
	virtual bool CancelApplicationLaunch(const char* pchAppKey) = 0;
	virtual EVRApplicationError IdentifyApplication(uint32_t unProcessId, const char* pchAppKey) = 0;
	virtual uint32_t GetApplicationProcessId(const char* pchAppKey) = 0;
	virtual const char* GetApplicationsErrorNameFromEnum(EVRApplicationError error) = 0;
	virtual uint32_t GetApplicationPropertyString(const char* pchAppKey, EVRApplicationProperty eProperty, VR_OUT_STRING() char* pchPropertyValueBuffer, uint32_t unPropertyValueBufferLen, EVRApplicationError* peError = nullptr) = 0;
	virtual bool GetApplicationPropertyBool(const char* pchAppKey, EVRApplicationProperty eProperty, EVRApplicationError* peError = nullptr) = 0;
	virtual uint64_t GetApplicationPropertyUint64(const char* pchAppKey, EVRApplicationProperty eProperty, EVRApplicationError* peError = nullptr) = 0;
	virtual EVRApplicationError SetApplicationAutoLaunch(const char* pchAppKey, bool bAutoLaunch) = 0;
	virtual bool GetApplicationAutoLaunch(const char* pchAppKey) = 0;
	virtual EVRApplicationError SetDefaultApplicationForMimeType(const char* pchAppKey, const char* pchMimeType) = 0;
	virtual bool GetDefaultApplicationForMimeType(const char* pchMimeType, VR_OUT_STRING() char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual bool GetApplicationSupportedMimeTypes(const char* pchAppKey, VR_OUT_STRING() char* pchMimeTypesBuffer, uint32_t unMimeTypesBuffer) = 0;
	virtual uint32_t GetApplicationsThatSupportMimeType(const char* pchMimeType, VR_OUT_STRING() char* pchAppKeysThatSupportBuffer, uint32_t unAppKeysThatSupportBuffer) = 0;
	virtual uint32_t GetApplicationLaunchArguments(uint32_t unHandle, VR_OUT_STRING() char* pchArgs, uint32_t unArgs) = 0;
	virtual EVRApplicationError GetStartingApplication(VR_OUT_STRING() char* pchAppKeyBuffer, uint32_t unAppKeyBufferLen) = 0;
	virtual EVRSceneApplicationState GetSceneApplicationState() = 0;
	virtual EVRApplicationError PerformApplicationPrelaunchCheck(const char* pchAppKey) = 0;
	virtual const char* GetSceneApplicationStateNameFromEnum(EVRSceneApplicationState state) = 0;
	virtual EVRApplicationError LaunchInternalProcess(const char* pchBinaryPath, const char* pchArguments, const char* pchWorkingDirectory) = 0;
	virtual EVRApplicationError RegisterSubprocess(uint32_t nPid) = 0;
	virtual uint32_t GetCurrentSceneProcessId() = 0;
};


static const char* const IVRChaperone_002_Version = "IVRChaperone_002";

class IVRChaperone_002
{
public:
	virtual ChaperoneCalibrationState GetCalibrationState() = 0;
	virtual bool GetSoftBoundsInfo(ChaperoneSoftBoundsInfo_t* pInfo) = 0;
	virtual bool GetHardBoundsInfo(VR_OUT_ARRAY_COUNT(punQuadsCount) HmdQuad_t* pQuadsBuffer, uint32_t* punQuadsCount) = 0;
	virtual bool GetSeatedBoundsInfo(ChaperoneSeatedBoundsInfo_t* pInfo) = 0;
	virtual void ReloadInfo(void) = 0;
	virtual void SetSceneColor(HmdColor_t color) = 0;
	virtual void GetBoundsColor(HmdColor_t* pOutputColorArray, int nNumOutputColors) = 0;
	virtual bool AreBoundsVisible() = 0;
	virtual void ForceBoundsVisible(bool bForce) = 0;
};


static const char* const IVRChaperone_003_Version = "IVRChaperone_003";

class IVRChaperone_003
{
public:
	virtual ChaperoneCalibrationState GetCalibrationState() = 0;
	virtual bool GetPlayAreaSize(float* pSizeX, float* pSizeZ) = 0;
	virtual bool GetPlayAreaRect(HmdQuad_t* rect) = 0;
	virtual void ReloadInfo(void) = 0;
	virtual void SetSceneColor(HmdColor_t color) = 0;
	virtual void GetBoundsColor(HmdColor_t* pOutputColorArray, int nNumOutputColors, float flCollisionBoundsFadeDistance, HmdColor_t* pOutputCameraColor) = 0;
	virtual bool AreBoundsVisible() = 0;
	virtual void ForceBoundsVisible(bool bForce) = 0;
};


static const char* const IVRChaperone_004_Version = "IVRChaperone_004";

class IVRChaperone_004
{
public:
	virtual ChaperoneCalibrationState GetCalibrationState() = 0;
	virtual bool GetPlayAreaSize(float* pSizeX, float* pSizeZ) = 0;
	virtual bool GetPlayAreaRect(HmdQuad_t* rect) = 0;
	virtual void ReloadInfo(void) = 0;
	virtual void SetSceneColor(HmdColor_t color) = 0;
	virtual void GetBoundsColor(HmdColor_t* pOutputColorArray, int nNumOutputColors, float flCollisionBoundsFadeDistance, HmdColor_t* pOutputCameraColor) = 0;
	virtual bool AreBoundsVisible() = 0;
	virtual void ForceBoundsVisible(bool bForce) = 0;
	virtual void ResetZeroPose(ETrackingUniverseOrigin eTrackingUniverseOrigin) = 0;
};


static const char* const IVRChaperoneSetup_004_Version = "IVRChaperoneSetup_004";

class IVRChaperoneSetup_004
{
public:
	virtual bool CommitWorkingCopy(EChaperoneConfigFile configFile) = 0;
	virtual void RevertWorkingCopy() = 0;
	virtual bool GetWorkingPlayAreaSize(float* pSizeX, float* pSizeZ) = 0;
	virtual bool GetWorkingPlayAreaRect(HmdQuad_t* rect) = 0;
	virtual bool GetWorkingCollisionBoundsInfo(VR_OUT_ARRAY_COUNT(punQuadsCount) HmdQuad_t* pQuadsBuffer, uint32_t* punQuadsCount) = 0;
	virtual bool GetLiveCollisionBoundsInfo(VR_OUT_ARRAY_COUNT(punQuadsCount) HmdQuad_t* pQuadsBuffer, uint32_t* punQuadsCount) = 0;
	virtual bool GetWorkingSeatedZeroPoseToRawTrackingPose(HmdMatrix34_t* pmatSeatedZeroPoseToRawTrackingPose) = 0;
	virtual bool GetWorkingStandingZeroPoseToRawTrackingPose(HmdMatrix34_t* pmatStandingZeroPoseToRawTrackingPose) = 0;
	virtual void SetWorkingPlayAreaSize(float sizeX, float sizeZ) = 0;
	virtual void SetWorkingCollisionBoundsInfo(VR_ARRAY_COUNT(unQuadsCount) HmdQuad_t* pQuadsBuffer, uint32_t unQuadsCount) = 0;
	virtual void SetWorkingSeatedZeroPoseToRawTrackingPose(const HmdMatrix34_t* pMatSeatedZeroPoseToRawTrackingPose) = 0;
	virtual void SetWorkingStandingZeroPoseToRawTrackingPose(const HmdMatrix34_t* pMatStandingZeroPoseToRawTrackingPose) = 0;
	virtual void ReloadFromDisk(EChaperoneConfigFile configFile) = 0;
	virtual bool GetLiveSeatedZeroPoseToRawTrackingPose(HmdMatrix34_t* pmatSeatedZeroPoseToRawTrackingPose) = 0;
	virtual void SetWorkingWallTagInfo(VR_ARRAY_COUNT(unTagCount) uint8_t* pTagsBuffer, uint32_t unTagCount) = 0;
	virtual bool GetLiveWallTagInfo(VR_OUT_ARRAY_COUNT(punTagCount) uint8_t* pTagsBuffer, uint32_t* punTagCount) = 0;
};


static const char* const IVRChaperoneSetup_005_Version = "IVRChaperoneSetup_005";

class IVRChaperoneSetup_005
{
public:
	virtual bool CommitWorkingCopy(EChaperoneConfigFile configFile) = 0;
	virtual void RevertWorkingCopy() = 0;
	virtual bool GetWorkingPlayAreaSize(float* pSizeX, float* pSizeZ) = 0;
	virtual bool GetWorkingPlayAreaRect(HmdQuad_t* rect) = 0;
	virtual bool GetWorkingCollisionBoundsInfo(VR_OUT_ARRAY_COUNT(punQuadsCount) HmdQuad_t* pQuadsBuffer, uint32_t* punQuadsCount) = 0;
	virtual bool GetLiveCollisionBoundsInfo(VR_OUT_ARRAY_COUNT(punQuadsCount) HmdQuad_t* pQuadsBuffer, uint32_t* punQuadsCount) = 0;
	virtual bool GetWorkingSeatedZeroPoseToRawTrackingPose(HmdMatrix34_t* pmatSeatedZeroPoseToRawTrackingPose) = 0;
	virtual bool GetWorkingStandingZeroPoseToRawTrackingPose(HmdMatrix34_t* pmatStandingZeroPoseToRawTrackingPose) = 0;
	virtual void SetWorkingPlayAreaSize(float sizeX, float sizeZ) = 0;
	virtual void SetWorkingCollisionBoundsInfo(VR_ARRAY_COUNT(unQuadsCount) HmdQuad_t* pQuadsBuffer, uint32_t unQuadsCount) = 0;
	virtual void SetWorkingSeatedZeroPoseToRawTrackingPose(const HmdMatrix34_t* pMatSeatedZeroPoseToRawTrackingPose) = 0;
	virtual void SetWorkingStandingZeroPoseToRawTrackingPose(const HmdMatrix34_t* pMatStandingZeroPoseToRawTrackingPose) = 0;
	virtual void ReloadFromDisk(EChaperoneConfigFile configFile) = 0;
	virtual bool GetLiveSeatedZeroPoseToRawTrackingPose(HmdMatrix34_t* pmatSeatedZeroPoseToRawTrackingPose) = 0;
	virtual void SetWorkingCollisionBoundsTagsInfo(VR_ARRAY_COUNT(unTagCount) uint8_t* pTagsBuffer, uint32_t unTagCount) = 0;
	virtual bool GetLiveCollisionBoundsTagsInfo(VR_OUT_ARRAY_COUNT(punTagCount) uint8_t* pTagsBuffer, uint32_t* punTagCount) = 0;
	virtual bool SetWorkingPhysicalBoundsInfo(VR_ARRAY_COUNT(unQuadsCount) HmdQuad_t* pQuadsBuffer, uint32_t unQuadsCount) = 0;
	virtual bool GetLivePhysicalBoundsInfo(VR_OUT_ARRAY_COUNT(punQuadsCount) HmdQuad_t* pQuadsBuffer, uint32_t* punQuadsCount) = 0;
	virtual bool ExportLiveToBuffer(VR_OUT_STRING() char* pBuffer, uint32_t* pnBufferLength) = 0;
	virtual bool ImportFromBufferToWorking(const char* pBuffer, uint32_t nImportFlags) = 0;
};


static const char* const IVRChaperoneSetup_006_Version = "IVRChaperoneSetup_006";

class IVRChaperoneSetup_006
{
public:
	virtual bool CommitWorkingCopy(EChaperoneConfigFile configFile) = 0;
	virtual void RevertWorkingCopy() = 0;
	virtual bool GetWorkingPlayAreaSize(float* pSizeX, float* pSizeZ) = 0;
	virtual bool GetWorkingPlayAreaRect(HmdQuad_t* rect) = 0;
	virtual bool GetWorkingCollisionBoundsInfo(VR_OUT_ARRAY_COUNT(punQuadsCount) HmdQuad_t* pQuadsBuffer, uint32_t* punQuadsCount) = 0;
	virtual bool GetLiveCollisionBoundsInfo(VR_OUT_ARRAY_COUNT(punQuadsCount) HmdQuad_t* pQuadsBuffer, uint32_t* punQuadsCount) = 0;
	virtual bool GetWorkingSeatedZeroPoseToRawTrackingPose(HmdMatrix34_t* pmatSeatedZeroPoseToRawTrackingPose) = 0;
	virtual bool GetWorkingStandingZeroPoseToRawTrackingPose(HmdMatrix34_t* pmatStandingZeroPoseToRawTrackingPose) = 0;
	virtual void SetWorkingPlayAreaSize(float sizeX, float sizeZ) = 0;
	virtual void SetWorkingCollisionBoundsInfo(VR_ARRAY_COUNT(unQuadsCount) HmdQuad_t* pQuadsBuffer, uint32_t unQuadsCount) = 0;
	virtual void SetWorkingPerimeter(VR_ARRAY_COUNT(unPointCount) const HmdVector2_t* pPointBuffer, uint32_t unPointCount) = 0;
	virtual void SetWorkingSeatedZeroPoseToRawTrackingPose(const HmdMatrix34_t* pMatSeatedZeroPoseToRawTrackingPose) = 0;
	virtual void SetWorkingStandingZeroPoseToRawTrackingPose(const HmdMatrix34_t* pMatStandingZeroPoseToRawTrackingPose) = 0;
	virtual void ReloadFromDisk(EChaperoneConfigFile configFile) = 0;
	virtual bool GetLiveSeatedZeroPoseToRawTrackingPose(HmdMatrix34_t* pmatSeatedZeroPoseToRawTrackingPose) = 0;
	virtual bool ExportLiveToBuffer(VR_OUT_STRING() char* pBuffer, uint32_t* pnBufferLength) = 0;
	virtual bool ImportFromBufferToWorking(const char* pBuffer, uint32_t nImportFlags) = 0;
	virtual void ShowWorkingSetPreview() = 0;
	virtual void HideWorkingSetPreview() = 0;
	virtual void RoomSetupStarting() = 0;
};


static const char* const IVRCompositor_005_Version = "IVRCompositor_005";

class IVRCompositor_005
{
public:
	virtual uint32_t GetLastError(VR_OUT_STRING() char* pchBuffer, uint32_t unBufferSize) = 0;
	virtual void SetVSync(bool bVSync) = 0;
	virtual bool GetVSync() = 0;
	virtual void SetGamma(float fGamma) = 0;
	virtual float GetGamma() = 0;
	virtual void SetGraphicsDevice(Compositor_DeviceType eType, void* pDevice) = 0;
	virtual void WaitGetPoses(VR_ARRAY_COUNT(unPoseArrayCount) TrackedDevicePose_t* pPoseArray, uint32_t unPoseArrayCount) = 0;
	virtual void Submit(Hmd_Eye eEye, void* pTexture, Compositor_TextureBounds* pBounds) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void GetOverlayDefaults(Compositor_OverlaySettings* pSettings) = 0;
	virtual void SetOverlay(void* pTexture, Compositor_OverlaySettings* pSettings) = 0;
	virtual void SetOverlayRaw(void* buffer, uint32_t width, uint32_t height, uint32_t depth, Compositor_OverlaySettings* pSettings) = 0;
	virtual void SetOverlayFromFile(const char* pchFilePath, Compositor_OverlaySettings* pSettings) = 0;
	virtual void ClearOverlay() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual bool ComputeOverlayIntersection(const Compositor_OverlaySettings* pSettings, float fAspectRatio, vr::TrackingUniverseOrigin eOrigin, vr::HmdVector3_t vSource, vr::HmdVector3_t vDirection, vr::HmdVector2_t* pvecIntersectionUV, vr::HmdVector3_t* pvecIntersectionTrackingSpace) = 0;
	virtual void SetTrackingSpace(TrackingUniverseOrigin eOrigin) = 0;
	virtual TrackingUniverseOrigin GetTrackingSpace() = 0;
};


static const char* const IVRCompositor_006_Version = "IVRCompositor_006";

class IVRCompositor_006
{
public:
	virtual uint32_t GetLastError(VR_OUT_STRING() char* pchBuffer, uint32_t unBufferSize) = 0;
	virtual void SetVSync(bool bVSync) = 0;
	virtual bool GetVSync() = 0;
	virtual void SetGamma(float fGamma) = 0;
	virtual float GetGamma() = 0;
	virtual void SetGraphicsDevice(Compositor_DeviceType eType, void* pDevice) = 0;
	virtual VRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual VRCompositorError Submit(Hmd_Eye eEye, void* pTexture, VRTextureBounds_t* pBounds) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual void SetTrackingSpace(TrackingUniverseOrigin eOrigin) = 0;
	virtual TrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual bool CanRenderScene() = 0;
};


static const char* const IVRCompositor_007_Version = "IVRCompositor_007";

class IVRCompositor_007
{
public:
	virtual uint32_t GetLastError(VR_OUT_STRING() char* pchBuffer, uint32_t unBufferSize) = 0;
	virtual void SetVSync(bool bVSync) = 0;
	virtual bool GetVSync() = 0;
	virtual void SetGamma(float fGamma) = 0;
	virtual float GetGamma() = 0;
	virtual VRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual VRCompositorError Submit(Hmd_Eye eEye, GraphicsAPIConvention eTextureType, void* pTexture, const VRTextureBounds_t* pBounds) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual void SetTrackingSpace(TrackingUniverseOrigin eOrigin) = 0;
	virtual TrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual bool CanRenderScene() = 0;
};


static const char* const IVRCompositor_008_Version = "IVRCompositor_008";

class IVRCompositor_008
{
public:
	virtual uint32_t GetLastError(VR_OUT_STRING() char* pchBuffer, uint32_t unBufferSize) = 0;
	virtual void SetVSync(bool bVSync) = 0;
	virtual bool GetVSync() = 0;
	virtual void SetGamma(float fGamma) = 0;
	virtual float GetGamma() = 0;
	virtual VRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual VRCompositorError Submit(Hmd_Eye eEye, GraphicsAPIConvention eTextureType, void* pTexture, const VRTextureBounds_t* pBounds, VRSubmitFlags_t nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual void SetSkyboxOverride(GraphicsAPIConvention eTextureType, void* pFront, void* pBack, void* pLeft, void* pRight, void* pTop, void* pBottom) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual void SetTrackingSpace(TrackingUniverseOrigin eOrigin) = 0;
	virtual TrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual VRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual void PostPresentHandoff() = 0;
};


static const char* const IVRCompositor_009_Version = "IVRCompositor_009";

class IVRCompositor_009
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
};


static const char* const IVRCompositor_010_Version = "IVRCompositor_010";

class IVRCompositor_010
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
};


static const char* const IVRCompositor_011_Version = "IVRCompositor_011";

class IVRCompositor_011
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
};


static const char* const IVRCompositor_012_Version = "IVRCompositor_012";

class IVRCompositor_012
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
};


static const char* const IVRCompositor_013_Version = "IVRCompositor_013";

class IVRCompositor_013
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
	virtual void ForceInterleavedReprojectionOn(bool bOverride) = 0;
};


static const char* const IVRCompositor_014_Version = "IVRCompositor_014";

class IVRCompositor_014
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
	virtual void ForceInterleavedReprojectionOn(bool bOverride) = 0;
	virtual void ForceReconnectProcess() = 0;
	virtual void SuspendRendering(bool bSuspend) = 0;
};


static const char* const IVRCompositor_015_Version = "IVRCompositor_015";

class IVRCompositor_015
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void GetCumulativeStats(Compositor_CumulativeStats* pStats, uint32_t nStatsSizeInBytes) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
	virtual void ForceInterleavedReprojectionOn(bool bOverride) = 0;
	virtual void ForceReconnectProcess() = 0;
	virtual void SuspendRendering(bool bSuspend) = 0;
	virtual vr::EVRCompositorError RequestScreenshot(vr::EVRScreenshotType type, const char* pchDestinationFileName, const char* pchVRDestinationFileName) = 0;
	virtual vr::EVRScreenshotType GetCurrentScreenshotType() = 0;
	virtual vr::EVRCompositorError GetMirrorTextureD3D11(vr::EVREye eEye, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureGL(vr::EVREye eEye, vr::glUInt_t* pglTextureId, vr::glSharedTextureHandle_t* pglSharedTextureHandle) = 0;
	virtual bool ReleaseSharedGLTexture(vr::glUInt_t glTextureId, vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void LockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void UnlockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
};


static const char* const IVRCompositor_016_Version = "IVRCompositor_016";

class IVRCompositor_016
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void GetCumulativeStats(Compositor_CumulativeStats* pStats, uint32_t nStatsSizeInBytes) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
	virtual void ForceInterleavedReprojectionOn(bool bOverride) = 0;
	virtual void ForceReconnectProcess() = 0;
	virtual void SuspendRendering(bool bSuspend) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureD3D11(vr::EVREye eEye, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureGL(vr::EVREye eEye, vr::glUInt_t* pglTextureId, vr::glSharedTextureHandle_t* pglSharedTextureHandle) = 0;
	virtual bool ReleaseSharedGLTexture(vr::glUInt_t glTextureId, vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void LockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void UnlockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
};


static const char* const IVRCompositor_018_Version = "IVRCompositor_018";

class IVRCompositor_018
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual uint32_t GetFrameTimings(Compositor_FrameTiming* pTiming, uint32_t nFrames) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void GetCumulativeStats(Compositor_CumulativeStats* pStats, uint32_t nStatsSizeInBytes) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual HmdColor_t GetCurrentFadeColor(bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual float GetCurrentGridAlpha() = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
	virtual void ForceInterleavedReprojectionOn(bool bOverride) = 0;
	virtual void ForceReconnectProcess() = 0;
	virtual void SuspendRendering(bool bSuspend) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureD3D11(vr::EVREye eEye, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureGL(vr::EVREye eEye, vr::glUInt_t* pglTextureId, vr::glSharedTextureHandle_t* pglSharedTextureHandle) = 0;
	virtual bool ReleaseSharedGLTexture(vr::glUInt_t glTextureId, vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void LockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void UnlockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
};


static const char* const IVRCompositor_019_Version = "IVRCompositor_019";

class IVRCompositor_019
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual uint32_t GetFrameTimings(Compositor_FrameTiming* pTiming, uint32_t nFrames) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void GetCumulativeStats(Compositor_CumulativeStats* pStats, uint32_t nStatsSizeInBytes) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual HmdColor_t GetCurrentFadeColor(bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual float GetCurrentGridAlpha() = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
	virtual void ForceInterleavedReprojectionOn(bool bOverride) = 0;
	virtual void ForceReconnectProcess() = 0;
	virtual void SuspendRendering(bool bSuspend) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureD3D11(vr::EVREye eEye, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureGL(vr::EVREye eEye, vr::glUInt_t* pglTextureId, vr::glSharedTextureHandle_t* pglSharedTextureHandle) = 0;
	virtual bool ReleaseSharedGLTexture(vr::glUInt_t glTextureId, vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void LockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void UnlockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual uint32_t GetVulkanInstanceExtensionsRequired(VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual uint32_t GetVulkanDeviceExtensionsRequired(VkPhysicalDevice_T* pPhysicalDevice, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
};


static const char* const IVRCompositor_020_Version = "IVRCompositor_020";

class IVRCompositor_020
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual uint32_t GetFrameTimings(Compositor_FrameTiming* pTiming, uint32_t nFrames) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void GetCumulativeStats(Compositor_CumulativeStats* pStats, uint32_t nStatsSizeInBytes) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual HmdColor_t GetCurrentFadeColor(bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual float GetCurrentGridAlpha() = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
	virtual void ForceInterleavedReprojectionOn(bool bOverride) = 0;
	virtual void ForceReconnectProcess() = 0;
	virtual void SuspendRendering(bool bSuspend) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureD3D11(vr::EVREye eEye, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView) = 0;
	virtual void ReleaseMirrorTextureD3D11(void* pD3D11ShaderResourceView) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureGL(vr::EVREye eEye, vr::glUInt_t* pglTextureId, vr::glSharedTextureHandle_t* pglSharedTextureHandle) = 0;
	virtual bool ReleaseSharedGLTexture(vr::glUInt_t glTextureId, vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void LockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void UnlockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual uint32_t GetVulkanInstanceExtensionsRequired(VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual uint32_t GetVulkanDeviceExtensionsRequired(VkPhysicalDevice_T* pPhysicalDevice, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
};


static const char* const IVRCompositor_021_Version = "IVRCompositor_021";

class IVRCompositor_021
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual uint32_t GetFrameTimings(Compositor_FrameTiming* pTiming, uint32_t nFrames) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void GetCumulativeStats(Compositor_CumulativeStats* pStats, uint32_t nStatsSizeInBytes) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual HmdColor_t GetCurrentFadeColor(bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual float GetCurrentGridAlpha() = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
	virtual void ForceInterleavedReprojectionOn(bool bOverride) = 0;
	virtual void ForceReconnectProcess() = 0;
	virtual void SuspendRendering(bool bSuspend) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureD3D11(vr::EVREye eEye, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView) = 0;
	virtual void ReleaseMirrorTextureD3D11(void* pD3D11ShaderResourceView) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureGL(vr::EVREye eEye, vr::glUInt_t* pglTextureId, vr::glSharedTextureHandle_t* pglSharedTextureHandle) = 0;
	virtual bool ReleaseSharedGLTexture(vr::glUInt_t glTextureId, vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void LockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void UnlockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual uint32_t GetVulkanInstanceExtensionsRequired(VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual uint32_t GetVulkanDeviceExtensionsRequired(VkPhysicalDevice_T* pPhysicalDevice, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual void SetExplicitTimingMode(bool bExplicitTimingMode) = 0;
	virtual EVRCompositorError SubmitExplicitTimingData() = 0;
};


static const char* const IVRCompositor_022_Version = "IVRCompositor_022";

class IVRCompositor_022
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual uint32_t GetFrameTimings(VR_ARRAY_COUNT(nFrames) Compositor_FrameTiming* pTiming, uint32_t nFrames) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void GetCumulativeStats(Compositor_CumulativeStats* pStats, uint32_t nStatsSizeInBytes) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual HmdColor_t GetCurrentFadeColor(bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual float GetCurrentGridAlpha() = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
	virtual void ForceInterleavedReprojectionOn(bool bOverride) = 0;
	virtual void ForceReconnectProcess() = 0;
	virtual void SuspendRendering(bool bSuspend) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureD3D11(vr::EVREye eEye, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView) = 0;
	virtual void ReleaseMirrorTextureD3D11(void* pD3D11ShaderResourceView) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureGL(vr::EVREye eEye, vr::glUInt_t* pglTextureId, vr::glSharedTextureHandle_t* pglSharedTextureHandle) = 0;
	virtual bool ReleaseSharedGLTexture(vr::glUInt_t glTextureId, vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void LockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void UnlockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual uint32_t GetVulkanInstanceExtensionsRequired(VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual uint32_t GetVulkanDeviceExtensionsRequired(VkPhysicalDevice_T* pPhysicalDevice, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual void SetExplicitTimingMode(EVRCompositorTimingMode eTimingMode) = 0;
	virtual EVRCompositorError SubmitExplicitTimingData() = 0;
	virtual bool IsMotionSmoothingEnabled() = 0;
	virtual bool IsMotionSmoothingSupported() = 0;
	virtual bool IsCurrentSceneFocusAppLoading() = 0;
};


static const char* const IVRCompositor_024_Version = "IVRCompositor_024";

class IVRCompositor_024
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual uint32_t GetFrameTimings(VR_ARRAY_COUNT(nFrames) Compositor_FrameTiming* pTiming, uint32_t nFrames) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void GetCumulativeStats(Compositor_CumulativeStats* pStats, uint32_t nStatsSizeInBytes) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual HmdColor_t GetCurrentFadeColor(bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual float GetCurrentGridAlpha() = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
	virtual void ForceInterleavedReprojectionOn(bool bOverride) = 0;
	virtual void ForceReconnectProcess() = 0;
	virtual void SuspendRendering(bool bSuspend) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureD3D11(vr::EVREye eEye, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView) = 0;
	virtual void ReleaseMirrorTextureD3D11(void* pD3D11ShaderResourceView) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureGL(vr::EVREye eEye, vr::glUInt_t* pglTextureId, vr::glSharedTextureHandle_t* pglSharedTextureHandle) = 0;
	virtual bool ReleaseSharedGLTexture(vr::glUInt_t glTextureId, vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void LockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void UnlockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual uint32_t GetVulkanInstanceExtensionsRequired(VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual uint32_t GetVulkanDeviceExtensionsRequired(VkPhysicalDevice_T* pPhysicalDevice, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual void SetExplicitTimingMode(EVRCompositorTimingMode eTimingMode) = 0;
	virtual EVRCompositorError SubmitExplicitTimingData() = 0;
	virtual bool IsMotionSmoothingEnabled() = 0;
	virtual bool IsMotionSmoothingSupported() = 0;
	virtual bool IsCurrentSceneFocusAppLoading() = 0;
	virtual EVRCompositorError SetStageOverride_Async(const char* pchRenderModelPath, const HmdMatrix34_t* pTransform = 0, const Compositor_StageRenderSettings* pRenderSettings = 0, uint32_t nSizeOfRenderSettings = 0) = 0;
	virtual void ClearStageOverride() = 0;
};


static const char* const IVRCompositor_026_Version = "IVRCompositor_026";

class IVRCompositor_026
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual uint32_t GetFrameTimings(VR_ARRAY_COUNT(nFrames) Compositor_FrameTiming* pTiming, uint32_t nFrames) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void GetCumulativeStats(Compositor_CumulativeStats* pStats, uint32_t nStatsSizeInBytes) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual HmdColor_t GetCurrentFadeColor(bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeIn) = 0;
	virtual float GetCurrentGridAlpha() = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
	virtual void ForceInterleavedReprojectionOn(bool bOverride) = 0;
	virtual void ForceReconnectProcess() = 0;
	virtual void SuspendRendering(bool bSuspend) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureD3D11(vr::EVREye eEye, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView) = 0;
	virtual void ReleaseMirrorTextureD3D11(void* pD3D11ShaderResourceView) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureGL(vr::EVREye eEye, vr::glUInt_t* pglTextureId, vr::glSharedTextureHandle_t* pglSharedTextureHandle) = 0;
	virtual bool ReleaseSharedGLTexture(vr::glUInt_t glTextureId, vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void LockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void UnlockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual uint32_t GetVulkanInstanceExtensionsRequired(VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual uint32_t GetVulkanDeviceExtensionsRequired(VkPhysicalDevice_T* pPhysicalDevice, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual void SetExplicitTimingMode(EVRCompositorTimingMode eTimingMode) = 0;
	virtual EVRCompositorError SubmitExplicitTimingData() = 0;
	virtual bool IsMotionSmoothingEnabled() = 0;
	virtual bool IsMotionSmoothingSupported() = 0;
	virtual bool IsCurrentSceneFocusAppLoading() = 0;
	virtual EVRCompositorError SetStageOverride_Async(const char* pchRenderModelPath, const HmdMatrix34_t* pTransform = 0, const Compositor_StageRenderSettings* pRenderSettings = 0, uint32_t nSizeOfRenderSettings = 0) = 0;
	virtual void ClearStageOverride() = 0;
	virtual bool GetCompositorBenchmarkResults(Compositor_BenchmarkResults* pBenchmarkResults, uint32_t nSizeOfBenchmarkResults) = 0;
	virtual EVRCompositorError GetLastPosePredictionIDs(uint32_t* pRenderPosePredictionID, uint32_t* pGamePosePredictionID) = 0;
	virtual EVRCompositorError GetPosesForFrame(uint32_t unPosePredictionID, VR_ARRAY_COUNT(unPoseArrayCount) TrackedDevicePose_t* pPoseArray, uint32_t unPoseArrayCount) = 0;
};


static const char* const IVRCompositor_027_Version = "IVRCompositor_027";

class IVRCompositor_027
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual uint32_t GetFrameTimings(VR_ARRAY_COUNT(nFrames) Compositor_FrameTiming* pTiming, uint32_t nFrames) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void GetCumulativeStats(Compositor_CumulativeStats* pStats, uint32_t nStatsSizeInBytes) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual HmdColor_t GetCurrentFadeColor(bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeGridIn) = 0;
	virtual float GetCurrentGridAlpha() = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
	virtual void ForceInterleavedReprojectionOn(bool bOverride) = 0;
	virtual void ForceReconnectProcess() = 0;
	virtual void SuspendRendering(bool bSuspend) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureD3D11(vr::EVREye eEye, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView) = 0;
	virtual void ReleaseMirrorTextureD3D11(void* pD3D11ShaderResourceView) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureGL(vr::EVREye eEye, vr::glUInt_t* pglTextureId, vr::glSharedTextureHandle_t* pglSharedTextureHandle) = 0;
	virtual bool ReleaseSharedGLTexture(vr::glUInt_t glTextureId, vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void LockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void UnlockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual uint32_t GetVulkanInstanceExtensionsRequired(VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual uint32_t GetVulkanDeviceExtensionsRequired(VkPhysicalDevice_T* pPhysicalDevice, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual void SetExplicitTimingMode(EVRCompositorTimingMode eTimingMode) = 0;
	virtual EVRCompositorError SubmitExplicitTimingData() = 0;
	virtual bool IsMotionSmoothingEnabled() = 0;
	virtual bool IsMotionSmoothingSupported() = 0;
	virtual bool IsCurrentSceneFocusAppLoading() = 0;
	virtual EVRCompositorError SetStageOverride_Async(const char* pchRenderModelPath, const HmdMatrix34_t* pTransform = 0, const Compositor_StageRenderSettings* pRenderSettings = 0, uint32_t nSizeOfRenderSettings = 0) = 0;
	virtual void ClearStageOverride() = 0;
	virtual bool GetCompositorBenchmarkResults(Compositor_BenchmarkResults* pBenchmarkResults, uint32_t nSizeOfBenchmarkResults) = 0;
	virtual EVRCompositorError GetLastPosePredictionIDs(uint32_t* pRenderPosePredictionID, uint32_t* pGamePosePredictionID) = 0;
	virtual EVRCompositorError GetPosesForFrame(uint32_t unPosePredictionID, VR_ARRAY_COUNT(unPoseArrayCount) TrackedDevicePose_t* pPoseArray, uint32_t unPoseArrayCount) = 0;
};


static const char* const IVRCompositor_028_Version = "IVRCompositor_028";

class IVRCompositor_028
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual EVRCompositorError SubmitWithArrayIndex(EVREye eEye, const Texture_t* pTexture, uint32_t unTextureArrayIndex, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual uint32_t GetFrameTimings(VR_ARRAY_COUNT(nFrames) Compositor_FrameTiming* pTiming, uint32_t nFrames) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void GetCumulativeStats(Compositor_CumulativeStats* pStats, uint32_t nStatsSizeInBytes) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual HmdColor_t GetCurrentFadeColor(bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeGridIn) = 0;
	virtual float GetCurrentGridAlpha() = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
	virtual void ForceInterleavedReprojectionOn(bool bOverride) = 0;
	virtual void ForceReconnectProcess() = 0;
	virtual void SuspendRendering(bool bSuspend) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureD3D11(vr::EVREye eEye, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView) = 0;
	virtual void ReleaseMirrorTextureD3D11(void* pD3D11ShaderResourceView) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureGL(vr::EVREye eEye, vr::glUInt_t* pglTextureId, vr::glSharedTextureHandle_t* pglSharedTextureHandle) = 0;
	virtual bool ReleaseSharedGLTexture(vr::glUInt_t glTextureId, vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void LockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void UnlockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual uint32_t GetVulkanInstanceExtensionsRequired(VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual uint32_t GetVulkanDeviceExtensionsRequired(VkPhysicalDevice_T* pPhysicalDevice, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual void SetExplicitTimingMode(EVRCompositorTimingMode eTimingMode) = 0;
	virtual EVRCompositorError SubmitExplicitTimingData() = 0;
	virtual bool IsMotionSmoothingEnabled() = 0;
	virtual bool IsMotionSmoothingSupported() = 0;
	virtual bool IsCurrentSceneFocusAppLoading() = 0;
	virtual EVRCompositorError SetStageOverride_Async(const char* pchRenderModelPath, const HmdMatrix34_t* pTransform = 0, const Compositor_StageRenderSettings* pRenderSettings = 0, uint32_t nSizeOfRenderSettings = 0) = 0;
	virtual void ClearStageOverride() = 0;
	virtual bool GetCompositorBenchmarkResults(Compositor_BenchmarkResults* pBenchmarkResults, uint32_t nSizeOfBenchmarkResults) = 0;
	virtual EVRCompositorError GetLastPosePredictionIDs(uint32_t* pRenderPosePredictionID, uint32_t* pGamePosePredictionID) = 0;
	virtual EVRCompositorError GetPosesForFrame(uint32_t unPosePredictionID, VR_ARRAY_COUNT(unPoseArrayCount) TrackedDevicePose_t* pPoseArray, uint32_t unPoseArrayCount) = 0;
};


static const char* const IVRCompositor_029_Version = "IVRCompositor_029";

class IVRCompositor_029
{
public:
	virtual void SetTrackingSpace(ETrackingUniverseOrigin eOrigin) = 0;
	virtual ETrackingUniverseOrigin GetTrackingSpace() = 0;
	virtual EVRCompositorError WaitGetPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoses(VR_ARRAY_COUNT(unRenderPoseArrayCount) TrackedDevicePose_t* pRenderPoseArray, uint32_t unRenderPoseArrayCount, VR_ARRAY_COUNT(unGamePoseArrayCount) TrackedDevicePose_t* pGamePoseArray, uint32_t unGamePoseArrayCount) = 0;
	virtual EVRCompositorError GetLastPoseForTrackedDeviceIndex(TrackedDeviceIndex_t unDeviceIndex, TrackedDevicePose_t* pOutputPose, TrackedDevicePose_t* pOutputGamePose) = 0;
	virtual EVRCompositorError GetSubmitTexture(Texture_t* pOutTexture, bool* pNeedsFlush, EVRCompositorTextureUsage eUsage, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual EVRCompositorError Submit(EVREye eEye, const Texture_t* pTexture, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual EVRCompositorError SubmitWithArrayIndex(EVREye eEye, const Texture_t* pTexture, uint32_t unTextureArrayIndex, const VRTextureBounds_t* pBounds = 0, EVRSubmitFlags nSubmitFlags = Submit_Default) = 0;
	virtual void ClearLastSubmittedFrame() = 0;
	virtual void PostPresentHandoff() = 0;
	virtual bool GetFrameTiming(Compositor_FrameTiming* pTiming, uint32_t unFramesAgo = 0) = 0;
	virtual uint32_t GetFrameTimings(VR_ARRAY_COUNT(nFrames) Compositor_FrameTiming* pTiming, uint32_t nFrames) = 0;
	virtual float GetFrameTimeRemaining() = 0;
	virtual void GetCumulativeStats(Compositor_CumulativeStats* pStats, uint32_t nStatsSizeInBytes) = 0;
	virtual void FadeToColor(float fSeconds, float fRed, float fGreen, float fBlue, float fAlpha, bool bBackground = false) = 0;
	virtual HmdColor_t GetCurrentFadeColor(bool bBackground = false) = 0;
	virtual void FadeGrid(float fSeconds, bool bFadeGridIn) = 0;
	virtual float GetCurrentGridAlpha() = 0;
	virtual EVRCompositorError SetSkyboxOverride(VR_ARRAY_COUNT(unTextureCount) const Texture_t* pTextures, uint32_t unTextureCount) = 0;
	virtual void ClearSkyboxOverride() = 0;
	virtual void CompositorBringToFront() = 0;
	virtual void CompositorGoToBack() = 0;
	virtual void CompositorQuit() = 0;
	virtual bool IsFullscreen() = 0;
	virtual uint32_t GetCurrentSceneFocusProcess() = 0;
	virtual uint32_t GetLastFrameRenderer() = 0;
	virtual bool CanRenderScene() = 0;
	virtual void ShowMirrorWindow() = 0;
	virtual void HideMirrorWindow() = 0;
	virtual bool IsMirrorWindowVisible() = 0;
	virtual void CompositorDumpImages() = 0;
	virtual bool ShouldAppRenderWithLowResources() = 0;
	virtual void ForceInterleavedReprojectionOn(bool bOverride) = 0;
	virtual void ForceReconnectProcess() = 0;
	virtual void SuspendRendering(bool bSuspend) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureD3D11(vr::EVREye eEye, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView) = 0;
	virtual void ReleaseMirrorTextureD3D11(void* pD3D11ShaderResourceView) = 0;
	virtual vr::EVRCompositorError GetMirrorTextureGL(vr::EVREye eEye, vr::glUInt_t* pglTextureId, vr::glSharedTextureHandle_t* pglSharedTextureHandle) = 0;
	virtual bool ReleaseSharedGLTexture(vr::glUInt_t glTextureId, vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void LockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual void UnlockGLSharedTextureForAccess(vr::glSharedTextureHandle_t glSharedTextureHandle) = 0;
	virtual uint32_t GetVulkanInstanceExtensionsRequired(VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual uint32_t GetVulkanDeviceExtensionsRequired(VkPhysicalDevice_T* pPhysicalDevice, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual void SetExplicitTimingMode(EVRCompositorTimingMode eTimingMode) = 0;
	virtual EVRCompositorError SubmitExplicitTimingData() = 0;
	virtual bool IsMotionSmoothingEnabled() = 0;
	virtual bool IsMotionSmoothingSupported() = 0;
	virtual bool IsCurrentSceneFocusAppLoading() = 0;
	virtual EVRCompositorError SetStageOverride_Async(const char* pchRenderModelPath, const HmdMatrix34_t* pTransform = 0, const Compositor_StageRenderSettings* pRenderSettings = 0, uint32_t nSizeOfRenderSettings = 0) = 0;
	virtual void ClearStageOverride() = 0;
	virtual bool GetCompositorBenchmarkResults(Compositor_BenchmarkResults* pBenchmarkResults, uint32_t nSizeOfBenchmarkResults) = 0;
	virtual EVRCompositorError GetLastPosePredictionIDs(uint32_t* pRenderPosePredictionID, uint32_t* pGamePosePredictionID) = 0;
	virtual EVRCompositorError GetPosesForFrame(uint32_t unPosePredictionID, VR_ARRAY_COUNT(unPoseArrayCount) TrackedDevicePose_t* pPoseArray, uint32_t unPoseArrayCount) = 0;
};


static const char* const IVRControlPanel_001_Version = "IVRControlPanel_001";

class IVRControlPanel_001
{
public:
	virtual uint32_t GetDriverCount() = 0;
	virtual uint32_t GetDriverId(uint32_t unDriverIndex, char* pchBuffer, uint32_t unBufferLen) = 0;
	virtual uint32_t GetDriverDisplayCount(const char* pchDriverId) = 0;
	virtual uint32_t GetDriverDisplayId(const char* pchDriverId, uint32_t unDisplayIndex, char* pchBuffer, uint32_t unBufferLen) = 0;
	virtual uint32_t GetDriverDisplayModelNumber(const char* pchDriverId, const char* pchDisplayId, char* pchBuffer, uint32_t unBufferLen) = 0;
	virtual uint32_t GetDriverDisplaySerialNumber(const char* pchDriverId, const char* pchDisplayId, char* pchBuffer, uint32_t unBufferLen) = 0;
	VR_IGNOREATTR()
	virtual class IVRSystem* GetCurrentDisplayInterface(const char* pchHmdInterfaceVersion) = 0;
	virtual uint32_t LoadSharedResource(const char* pchResourceName, char* pchBuffer, uint32_t unBufferLen) = 0;
	virtual float GetIPD() = 0;
	virtual void SetIPD(float fIPD) = 0;
	virtual class vr::IVRCompositor* GetCurrentCompositorInterface(const char* pchInterfaceVersion) = 0;
	virtual bool QuitProcess(uint32_t pidProcessToQuit) = 0;
	virtual uint32_t StartVRProcess(const char* pchExecutable, const char** pchArguments, uint32_t unArgumentCount, const char* pchWorkingDirectory) = 0;
	virtual void SetMasterProcessToThis() = 0;
	virtual void StartAutolaunchOverlays() = 0;
	virtual bool ForceQuitProcess(uint32_t pidProcessToQuit) = 0;
	virtual void AbortTransition() = 0;
};


static const char* const IVRDebug_001_Version = "IVRDebug_001";

class IVRDebug_001
{
public:
	virtual EVRDebugError EmitVrProfilerEvent(const char* pchMessage) = 0;
	virtual EVRDebugError BeginVrProfilerEvent(VrProfilerEventHandle_t* pHandleOut) = 0;
	virtual EVRDebugError FinishVrProfilerEvent(VrProfilerEventHandle_t hHandle, const char* pchMessage) = 0;
	virtual uint32_t DriverDebugRequest(vr::TrackedDeviceIndex_t unDeviceIndex, const char* pchRequest, VR_OUT_STRING() char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
};


static const char* const IVRDriverManager_001_Version = "IVRDriverManager_001";

class IVRDriverManager_001
{
public:
	virtual uint32_t GetDriverCount() const = 0;
	virtual uint32_t GetDriverName(vr::DriverId_t nDriver, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize) = 0;
	virtual DriverHandle_t GetDriverHandle(const char* pchDriverName) = 0;
	virtual bool IsEnabled(vr::DriverId_t nDriver) const = 0;
};


static const char* const IVRExtendedDisplay_001_Version = "IVRExtendedDisplay_001";

class IVRExtendedDisplay_001
{
public:
	virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual void GetEyeOutputViewport(EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex, int32_t* pnAdapterOutputIndex) = 0;
};


static const char* const IVRHeadsetView_001_Version = "IVRHeadsetView_001";

class IVRHeadsetView_001
{
public:
	virtual void SetHeadsetViewSize(uint32_t nWidth, uint32_t nHeight) = 0;
	virtual void GetHeadsetViewSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual void SetHeadsetViewMode(HeadsetViewMode_t eHeadsetViewMode) = 0;
	virtual HeadsetViewMode_t GetHeadsetViewMode() = 0;
	virtual void SetHeadsetViewCropped(bool bCropped) = 0;
	virtual bool GetHeadsetViewCropped() = 0;
	virtual float GetHeadsetViewAspectRatio() = 0;
	virtual void SetHeadsetViewBlendRange(float flStartPct, float flEndPct) = 0;
	virtual void GetHeadsetViewBlendRange(float* pStartPct, float* pEndPct) = 0;
};


static const char* const IVRInput_003_Version = "IVRInput_003";

class IVRInput_003
{
public:
	virtual EVRInputError SetActionManifestPath(const char* pchActionManifestPath) = 0;
	virtual EVRInputError GetActionSetHandle(const char* pchActionSetName, VRActionSetHandle_t* pHandle) = 0;
	virtual EVRInputError GetActionHandle(const char* pchActionName, VRActionHandle_t* pHandle) = 0;
	virtual EVRInputError GetInputSourceHandle(const char* pchInputSourcePath, VRInputValueHandle_t* pHandle) = 0;
	virtual EVRInputError UpdateActionState(VR_ARRAY_COUNT(unSetCount) VRActiveActionSet_t* pSets, uint32_t unSizeOfVRSelectedActionSet_t, uint32_t unSetCount) = 0;
	virtual EVRInputError GetDigitalActionData(VRActionHandle_t action, InputDigitalActionData_t* pActionData, uint32_t unActionDataSize) = 0;
	virtual EVRInputError GetAnalogActionData(VRActionHandle_t action, InputAnalogActionData_t* pActionData, uint32_t unActionDataSize) = 0;
	virtual EVRInputError GetPoseActionData(VRActionHandle_t action, ETrackingUniverseOrigin eOrigin, float fPredictedSecondsFromNow, InputPoseActionData_t* pActionData, uint32_t unActionDataSize) = 0;
	virtual EVRInputError GetSkeletalActionData(VRActionHandle_t action, EVRSkeletalTransformSpace eBoneParent, float fPredictedSecondsFromNow, InputSkeletonActionData_t* pActionData, uint32_t unActionDataSize, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError GetSkeletalActionDataCompressed(VRActionHandle_t action, EVRSkeletalTransformSpace eBoneParent, float fPredictedSecondsFromNow, VR_OUT_BUFFER_COUNT(unCompressedSize) void* pvCompressedData, uint32_t unCompressedSize, uint32_t* punRequiredCompressedSize) = 0;
	virtual EVRInputError UncompressSkeletalActionData(void* pvCompressedBuffer, uint32_t unCompressedBufferSize, EVRSkeletalTransformSpace* peBoneParent, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError TriggerHapticVibrationAction(VRActionHandle_t action, float fStartSecondsFromNow, float fDurationSeconds, float fFrequency, float fAmplitude) = 0;
	virtual EVRInputError GetActionOrigins(VRActionSetHandle_t actionSetHandle, VRActionHandle_t digitalActionHandle, VR_ARRAY_COUNT(originOutCount) VRInputValueHandle_t* originsOut, uint32_t originOutCount) = 0;
	virtual EVRInputError GetOriginLocalizedName(VRInputValueHandle_t origin, VR_OUT_STRING() char* pchNameArray, uint32_t unNameArraySize) = 0;
	virtual EVRInputError GetOriginTrackedDeviceInfo(VRInputValueHandle_t origin, InputOriginInfo_t* pOriginInfo, uint32_t unOriginInfoSize) = 0;
	virtual EVRInputError ShowActionOrigins(VRActionSetHandle_t actionSetHandle, VRActionHandle_t ulActionHandle) = 0;
	virtual EVRInputError ShowBindingsForActionSet(VR_ARRAY_COUNT(unSetCount) VRActiveActionSet_t* pSets, uint32_t unSizeOfVRSelectedActionSet_t, uint32_t unSetCount, VRInputValueHandle_t originToHighlight) = 0;
};


static const char* const IVRInput_004_Version = "IVRInput_004";

class IVRInput_004
{
public:
	virtual EVRInputError SetActionManifestPath(const char* pchActionManifestPath) = 0;
	virtual EVRInputError GetActionSetHandle(const char* pchActionSetName, VRActionSetHandle_t* pHandle) = 0;
	virtual EVRInputError GetActionHandle(const char* pchActionName, VRActionHandle_t* pHandle) = 0;
	virtual EVRInputError GetInputSourceHandle(const char* pchInputSourcePath, VRInputValueHandle_t* pHandle) = 0;
	virtual EVRInputError UpdateActionState(VR_ARRAY_COUNT(unSetCount) VRActiveActionSet_t* pSets, uint32_t unSizeOfVRSelectedActionSet_t, uint32_t unSetCount) = 0;
	virtual EVRInputError GetDigitalActionData(VRActionHandle_t action, InputDigitalActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetAnalogActionData(VRActionHandle_t action, InputAnalogActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetPoseActionData(VRActionHandle_t action, ETrackingUniverseOrigin eOrigin, float fPredictedSecondsFromNow, InputPoseActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetSkeletalActionData(VRActionHandle_t action, InputSkeletalActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetSkeletalBoneData(VRActionHandle_t action, EVRSkeletalTransformSpace eTransformSpace, EVRSkeletalMotionRange eMotionRange, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetSkeletalBoneDataCompressed(VRActionHandle_t action, EVRSkeletalTransformSpace eTransformSpace, EVRSkeletalMotionRange eMotionRange, VR_OUT_BUFFER_COUNT(unCompressedSize) void* pvCompressedData, uint32_t unCompressedSize, uint32_t* punRequiredCompressedSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError DecompressSkeletalBoneData(void* pvCompressedBuffer, uint32_t unCompressedBufferSize, EVRSkeletalTransformSpace* peTransformSpace, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError TriggerHapticVibrationAction(VRActionHandle_t action, float fStartSecondsFromNow, float fDurationSeconds, float fFrequency, float fAmplitude, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetActionOrigins(VRActionSetHandle_t actionSetHandle, VRActionHandle_t digitalActionHandle, VR_ARRAY_COUNT(originOutCount) VRInputValueHandle_t* originsOut, uint32_t originOutCount) = 0;
	virtual EVRInputError GetOriginLocalizedName(VRInputValueHandle_t origin, VR_OUT_STRING() char* pchNameArray, uint32_t unNameArraySize) = 0;
	virtual EVRInputError GetOriginTrackedDeviceInfo(VRInputValueHandle_t origin, InputOriginInfo_t* pOriginInfo, uint32_t unOriginInfoSize) = 0;
	virtual EVRInputError ShowActionOrigins(VRActionSetHandle_t actionSetHandle, VRActionHandle_t ulActionHandle) = 0;
	virtual EVRInputError ShowBindingsForActionSet(VR_ARRAY_COUNT(unSetCount) VRActiveActionSet_t* pSets, uint32_t unSizeOfVRSelectedActionSet_t, uint32_t unSetCount, VRInputValueHandle_t originToHighlight) = 0;
};


static const char* const IVRInput_005_Version = "IVRInput_005";

class IVRInput_005
{
public:
	virtual EVRInputError SetActionManifestPath(const char* pchActionManifestPath) = 0;
	virtual EVRInputError GetActionSetHandle(const char* pchActionSetName, VRActionSetHandle_t* pHandle) = 0;
	virtual EVRInputError GetActionHandle(const char* pchActionName, VRActionHandle_t* pHandle) = 0;
	virtual EVRInputError GetInputSourceHandle(const char* pchInputSourcePath, VRInputValueHandle_t* pHandle) = 0;
	virtual EVRInputError UpdateActionState(VR_ARRAY_COUNT(unSetCount) VRActiveActionSet_t* pSets, uint32_t unSizeOfVRSelectedActionSet_t, uint32_t unSetCount) = 0;
	virtual EVRInputError GetDigitalActionData(VRActionHandle_t action, InputDigitalActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetAnalogActionData(VRActionHandle_t action, InputAnalogActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetPoseActionData(VRActionHandle_t action, ETrackingUniverseOrigin eOrigin, float fPredictedSecondsFromNow, InputPoseActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetSkeletalActionData(VRActionHandle_t action, InputSkeletalActionData_t* pActionData, uint32_t unActionDataSize) = 0;
	virtual EVRInputError GetBoneCount(VRActionHandle_t action, uint32_t* pBoneCount) = 0;
	virtual EVRInputError GetBoneHierarchy(VRActionHandle_t action, VR_ARRAY_COUNT(unIndexArayCount) BoneIndex_t* pParentIndices, uint32_t unIndexArayCount) = 0;
	virtual EVRInputError GetBoneName(VRActionHandle_t action, BoneIndex_t nBoneIndex, VR_OUT_STRING() char* pchBoneName, uint32_t unNameBufferSize) = 0;
	virtual EVRInputError GetSkeletalReferenceTransforms(VRActionHandle_t action, EVRSkeletalTransformSpace eTransformSpace, EVRSkeletalReferencePose eReferencePose, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError GetSkeletalTrackingLevel(VRActionHandle_t action, EVRSkeletalTrackingLevel* pSkeletalTrackingLevel) = 0;
	virtual EVRInputError GetSkeletalBoneData(VRActionHandle_t action, EVRSkeletalTransformSpace eTransformSpace, EVRSkeletalMotionRange eMotionRange, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError GetSkeletalSummaryData(VRActionHandle_t action, VRSkeletalSummaryData_t* pSkeletalSummaryData) = 0;
	virtual EVRInputError GetSkeletalBoneDataCompressed(VRActionHandle_t action, EVRSkeletalMotionRange eMotionRange, VR_OUT_BUFFER_COUNT(unCompressedSize) void* pvCompressedData, uint32_t unCompressedSize, uint32_t* punRequiredCompressedSize) = 0;
	virtual EVRInputError DecompressSkeletalBoneData(const void* pvCompressedBuffer, uint32_t unCompressedBufferSize, EVRSkeletalTransformSpace eTransformSpace, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError TriggerHapticVibrationAction(VRActionHandle_t action, float fStartSecondsFromNow, float fDurationSeconds, float fFrequency, float fAmplitude, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetActionOrigins(VRActionSetHandle_t actionSetHandle, VRActionHandle_t digitalActionHandle, VR_ARRAY_COUNT(originOutCount) VRInputValueHandle_t* originsOut, uint32_t originOutCount) = 0;
	virtual EVRInputError GetOriginLocalizedName(VRInputValueHandle_t origin, VR_OUT_STRING() char* pchNameArray, uint32_t unNameArraySize, int32_t unStringSectionsToInclude) = 0;
	virtual EVRInputError GetOriginTrackedDeviceInfo(VRInputValueHandle_t origin, InputOriginInfo_t* pOriginInfo, uint32_t unOriginInfoSize) = 0;
	virtual EVRInputError ShowActionOrigins(VRActionSetHandle_t actionSetHandle, VRActionHandle_t ulActionHandle) = 0;
	virtual EVRInputError ShowBindingsForActionSet(VR_ARRAY_COUNT(unSetCount) VRActiveActionSet_t* pSets, uint32_t unSizeOfVRSelectedActionSet_t, uint32_t unSetCount, VRInputValueHandle_t originToHighlight) = 0;
	virtual bool IsUsingLegacyInput() = 0;
};


static const char* const IVRInput_006_Version = "IVRInput_006";

class IVRInput_006
{
public:
	virtual EVRInputError SetActionManifestPath(const char* pchActionManifestPath) = 0;
	virtual EVRInputError GetActionSetHandle(const char* pchActionSetName, VRActionSetHandle_t* pHandle) = 0;
	virtual EVRInputError GetActionHandle(const char* pchActionName, VRActionHandle_t* pHandle) = 0;
	virtual EVRInputError GetInputSourceHandle(const char* pchInputSourcePath, VRInputValueHandle_t* pHandle) = 0;
	virtual EVRInputError UpdateActionState(VR_ARRAY_COUNT(unSetCount) VRActiveActionSet_t* pSets, uint32_t unSizeOfVRSelectedActionSet_t, uint32_t unSetCount) = 0;
	virtual EVRInputError GetDigitalActionData(VRActionHandle_t action, InputDigitalActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetAnalogActionData(VRActionHandle_t action, InputAnalogActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetPoseActionDataRelativeToNow(VRActionHandle_t action, ETrackingUniverseOrigin eOrigin, float fPredictedSecondsFromNow, InputPoseActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetPoseActionDataForNextFrame(VRActionHandle_t action, ETrackingUniverseOrigin eOrigin, InputPoseActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetSkeletalActionData(VRActionHandle_t action, InputSkeletalActionData_t* pActionData, uint32_t unActionDataSize) = 0;
	virtual EVRInputError GetBoneCount(VRActionHandle_t action, uint32_t* pBoneCount) = 0;
	virtual EVRInputError GetBoneHierarchy(VRActionHandle_t action, VR_ARRAY_COUNT(unIndexArayCount) BoneIndex_t* pParentIndices, uint32_t unIndexArayCount) = 0;
	virtual EVRInputError GetBoneName(VRActionHandle_t action, BoneIndex_t nBoneIndex, VR_OUT_STRING() char* pchBoneName, uint32_t unNameBufferSize) = 0;
	virtual EVRInputError GetSkeletalReferenceTransforms(VRActionHandle_t action, EVRSkeletalTransformSpace eTransformSpace, EVRSkeletalReferencePose eReferencePose, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError GetSkeletalTrackingLevel(VRActionHandle_t action, EVRSkeletalTrackingLevel* pSkeletalTrackingLevel) = 0;
	virtual EVRInputError GetSkeletalBoneData(VRActionHandle_t action, EVRSkeletalTransformSpace eTransformSpace, EVRSkeletalMotionRange eMotionRange, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError GetSkeletalSummaryData(VRActionHandle_t action, EVRSummaryType eSummaryType, VRSkeletalSummaryData_t* pSkeletalSummaryData) = 0;
	virtual EVRInputError GetSkeletalBoneDataCompressed(VRActionHandle_t action, EVRSkeletalMotionRange eMotionRange, VR_OUT_BUFFER_COUNT(unCompressedSize) void* pvCompressedData, uint32_t unCompressedSize, uint32_t* punRequiredCompressedSize) = 0;
	virtual EVRInputError DecompressSkeletalBoneData(const void* pvCompressedBuffer, uint32_t unCompressedBufferSize, EVRSkeletalTransformSpace eTransformSpace, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError TriggerHapticVibrationAction(VRActionHandle_t action, float fStartSecondsFromNow, float fDurationSeconds, float fFrequency, float fAmplitude, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetActionOrigins(VRActionSetHandle_t actionSetHandle, VRActionHandle_t digitalActionHandle, VR_ARRAY_COUNT(originOutCount) VRInputValueHandle_t* originsOut, uint32_t originOutCount) = 0;
	virtual EVRInputError GetOriginLocalizedName(VRInputValueHandle_t origin, VR_OUT_STRING() char* pchNameArray, uint32_t unNameArraySize, int32_t unStringSectionsToInclude) = 0;
	virtual EVRInputError GetOriginTrackedDeviceInfo(VRInputValueHandle_t origin, InputOriginInfo_t* pOriginInfo, uint32_t unOriginInfoSize) = 0;
	virtual EVRInputError ShowActionOrigins(VRActionSetHandle_t actionSetHandle, VRActionHandle_t ulActionHandle) = 0;
	virtual EVRInputError ShowBindingsForActionSet(VR_ARRAY_COUNT(unSetCount) VRActiveActionSet_t* pSets, uint32_t unSizeOfVRSelectedActionSet_t, uint32_t unSetCount, VRInputValueHandle_t originToHighlight) = 0;
	virtual bool IsUsingLegacyInput() = 0;
};


static const char* const IVRInput_007_Version = "IVRInput_007";

class IVRInput_007
{
public:
	virtual EVRInputError SetActionManifestPath(const char* pchActionManifestPath) = 0;
	virtual EVRInputError GetActionSetHandle(const char* pchActionSetName, VRActionSetHandle_t* pHandle) = 0;
	virtual EVRInputError GetActionHandle(const char* pchActionName, VRActionHandle_t* pHandle) = 0;
	virtual EVRInputError GetInputSourceHandle(const char* pchInputSourcePath, VRInputValueHandle_t* pHandle) = 0;
	virtual EVRInputError UpdateActionState(VR_ARRAY_COUNT(unSetCount) VRActiveActionSet_t* pSets, uint32_t unSizeOfVRSelectedActionSet_t, uint32_t unSetCount) = 0;
	virtual EVRInputError GetDigitalActionData(VRActionHandle_t action, InputDigitalActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetAnalogActionData(VRActionHandle_t action, InputAnalogActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetPoseActionDataRelativeToNow(VRActionHandle_t action, ETrackingUniverseOrigin eOrigin, float fPredictedSecondsFromNow, InputPoseActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetPoseActionDataForNextFrame(VRActionHandle_t action, ETrackingUniverseOrigin eOrigin, InputPoseActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetSkeletalActionData(VRActionHandle_t action, InputSkeletalActionData_t* pActionData, uint32_t unActionDataSize) = 0;
	virtual EVRInputError GetBoneCount(VRActionHandle_t action, uint32_t* pBoneCount) = 0;
	virtual EVRInputError GetBoneHierarchy(VRActionHandle_t action, VR_ARRAY_COUNT(unIndexArayCount) BoneIndex_t* pParentIndices, uint32_t unIndexArayCount) = 0;
	virtual EVRInputError GetBoneName(VRActionHandle_t action, BoneIndex_t nBoneIndex, VR_OUT_STRING() char* pchBoneName, uint32_t unNameBufferSize) = 0;
	virtual EVRInputError GetSkeletalReferenceTransforms(VRActionHandle_t action, EVRSkeletalTransformSpace eTransformSpace, EVRSkeletalReferencePose eReferencePose, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError GetSkeletalTrackingLevel(VRActionHandle_t action, EVRSkeletalTrackingLevel* pSkeletalTrackingLevel) = 0;
	virtual EVRInputError GetSkeletalBoneData(VRActionHandle_t action, EVRSkeletalTransformSpace eTransformSpace, EVRSkeletalMotionRange eMotionRange, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError GetSkeletalSummaryData(VRActionHandle_t action, EVRSummaryType eSummaryType, VRSkeletalSummaryData_t* pSkeletalSummaryData) = 0;
	virtual EVRInputError GetSkeletalBoneDataCompressed(VRActionHandle_t action, EVRSkeletalMotionRange eMotionRange, VR_OUT_BUFFER_COUNT(unCompressedSize) void* pvCompressedData, uint32_t unCompressedSize, uint32_t* punRequiredCompressedSize) = 0;
	virtual EVRInputError DecompressSkeletalBoneData(const void* pvCompressedBuffer, uint32_t unCompressedBufferSize, EVRSkeletalTransformSpace eTransformSpace, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError TriggerHapticVibrationAction(VRActionHandle_t action, float fStartSecondsFromNow, float fDurationSeconds, float fFrequency, float fAmplitude, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetActionOrigins(VRActionSetHandle_t actionSetHandle, VRActionHandle_t digitalActionHandle, VR_ARRAY_COUNT(originOutCount) VRInputValueHandle_t* originsOut, uint32_t originOutCount) = 0;
	virtual EVRInputError GetOriginLocalizedName(VRInputValueHandle_t origin, VR_OUT_STRING() char* pchNameArray, uint32_t unNameArraySize, int32_t unStringSectionsToInclude) = 0;
	virtual EVRInputError GetOriginTrackedDeviceInfo(VRInputValueHandle_t origin, InputOriginInfo_t* pOriginInfo, uint32_t unOriginInfoSize) = 0;
	virtual EVRInputError GetActionBindingInfo(VRActionHandle_t action, InputBindingInfo_t* pOriginInfo, uint32_t unBindingInfoSize, uint32_t unBindingInfoCount, uint32_t* punReturnedBindingInfoCount) = 0;
	virtual EVRInputError ShowActionOrigins(VRActionSetHandle_t actionSetHandle, VRActionHandle_t ulActionHandle) = 0;
	virtual EVRInputError ShowBindingsForActionSet(VR_ARRAY_COUNT(unSetCount) VRActiveActionSet_t* pSets, uint32_t unSizeOfVRSelectedActionSet_t, uint32_t unSetCount, VRInputValueHandle_t originToHighlight) = 0;
	virtual bool IsUsingLegacyInput() = 0;
	virtual EVRInputError OpenBindingUI(const char* pchAppKey, VRActionSetHandle_t ulActionSetHandle, VRInputValueHandle_t ulDeviceHandle, bool bShowOnDesktop) = 0;
};


static const char* const IVRInput_010_Version = "IVRInput_010";

class IVRInput_010
{
public:
	virtual EVRInputError SetActionManifestPath(const char* pchActionManifestPath) = 0;
	virtual EVRInputError GetActionSetHandle(const char* pchActionSetName, VRActionSetHandle_t* pHandle) = 0;
	virtual EVRInputError GetActionHandle(const char* pchActionName, VRActionHandle_t* pHandle) = 0;
	virtual EVRInputError GetInputSourceHandle(const char* pchInputSourcePath, VRInputValueHandle_t* pHandle) = 0;
	virtual EVRInputError UpdateActionState(VR_ARRAY_COUNT(unSetCount) VRActiveActionSet_t* pSets, uint32_t unSizeOfVRSelectedActionSet_t, uint32_t unSetCount) = 0;
	virtual EVRInputError GetDigitalActionData(VRActionHandle_t action, InputDigitalActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetAnalogActionData(VRActionHandle_t action, InputAnalogActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetPoseActionDataRelativeToNow(VRActionHandle_t action, ETrackingUniverseOrigin eOrigin, float fPredictedSecondsFromNow, InputPoseActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetPoseActionDataForNextFrame(VRActionHandle_t action, ETrackingUniverseOrigin eOrigin, InputPoseActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetSkeletalActionData(VRActionHandle_t action, InputSkeletalActionData_t* pActionData, uint32_t unActionDataSize) = 0;
	virtual EVRInputError GetDominantHand(ETrackedControllerRole* peDominantHand) = 0;
	virtual EVRInputError SetDominantHand(ETrackedControllerRole eDominantHand) = 0;
	virtual EVRInputError GetBoneCount(VRActionHandle_t action, uint32_t* pBoneCount) = 0;
	virtual EVRInputError GetBoneHierarchy(VRActionHandle_t action, VR_ARRAY_COUNT(unIndexArayCount) BoneIndex_t* pParentIndices, uint32_t unIndexArayCount) = 0;
	virtual EVRInputError GetBoneName(VRActionHandle_t action, BoneIndex_t nBoneIndex, VR_OUT_STRING() char* pchBoneName, uint32_t unNameBufferSize) = 0;
	virtual EVRInputError GetSkeletalReferenceTransforms(VRActionHandle_t action, EVRSkeletalTransformSpace eTransformSpace, EVRSkeletalReferencePose eReferencePose, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError GetSkeletalTrackingLevel(VRActionHandle_t action, EVRSkeletalTrackingLevel* pSkeletalTrackingLevel) = 0;
	virtual EVRInputError GetSkeletalBoneData(VRActionHandle_t action, EVRSkeletalTransformSpace eTransformSpace, EVRSkeletalMotionRange eMotionRange, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError GetSkeletalSummaryData(VRActionHandle_t action, EVRSummaryType eSummaryType, VRSkeletalSummaryData_t* pSkeletalSummaryData) = 0;
	virtual EVRInputError GetSkeletalBoneDataCompressed(VRActionHandle_t action, EVRSkeletalMotionRange eMotionRange, VR_OUT_BUFFER_COUNT(unCompressedSize) void* pvCompressedData, uint32_t unCompressedSize, uint32_t* punRequiredCompressedSize) = 0;
	virtual EVRInputError DecompressSkeletalBoneData(const void* pvCompressedBuffer, uint32_t unCompressedBufferSize, EVRSkeletalTransformSpace eTransformSpace, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError TriggerHapticVibrationAction(VRActionHandle_t action, float fStartSecondsFromNow, float fDurationSeconds, float fFrequency, float fAmplitude, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetActionOrigins(VRActionSetHandle_t actionSetHandle, VRActionHandle_t digitalActionHandle, VR_ARRAY_COUNT(originOutCount) VRInputValueHandle_t* originsOut, uint32_t originOutCount) = 0;
	virtual EVRInputError GetOriginLocalizedName(VRInputValueHandle_t origin, VR_OUT_STRING() char* pchNameArray, uint32_t unNameArraySize, int32_t unStringSectionsToInclude) = 0;
	virtual EVRInputError GetOriginTrackedDeviceInfo(VRInputValueHandle_t origin, InputOriginInfo_t* pOriginInfo, uint32_t unOriginInfoSize) = 0;
	virtual EVRInputError GetActionBindingInfo(VRActionHandle_t action, VR_ARRAY_COUNT(unBindingInfoCount) InputBindingInfo_t* pOriginInfo, uint32_t unBindingInfoSize, uint32_t unBindingInfoCount, uint32_t* punReturnedBindingInfoCount) = 0;
	virtual EVRInputError ShowActionOrigins(VRActionSetHandle_t actionSetHandle, VRActionHandle_t ulActionHandle) = 0;
	virtual EVRInputError ShowBindingsForActionSet(VR_ARRAY_COUNT(unSetCount) VRActiveActionSet_t* pSets, uint32_t unSizeOfVRSelectedActionSet_t, uint32_t unSetCount, VRInputValueHandle_t originToHighlight) = 0;
	virtual EVRInputError GetComponentStateForBinding(const char* pchRenderModelName, const char* pchComponentName, const InputBindingInfo_t* pOriginInfo, uint32_t unBindingInfoSize, uint32_t unBindingInfoCount, vr::RenderModel_ComponentState_t* pComponentState) = 0;
	virtual bool IsUsingLegacyInput() = 0;
	virtual EVRInputError OpenBindingUI(const char* pchAppKey, VRActionSetHandle_t ulActionSetHandle, VRInputValueHandle_t ulDeviceHandle, bool bShowOnDesktop) = 0;
	virtual EVRInputError GetBindingVariant(vr::VRInputValueHandle_t ulDevicePath, VR_OUT_STRING() char* pchVariantArray, uint32_t unVariantArraySize) = 0;
};


static const char* const IVRInput_011_Version = "IVRInput_011";

class IVRInput_011
{
public:
	virtual EVRInputError SetActionManifestPath(const char* pchActionManifestPath) = 0;
	virtual EVRInputError GetActionSetHandle(const char* pchActionSetName, VRActionSetHandle_t* pHandle) = 0;
	virtual EVRInputError GetActionHandle(const char* pchActionName, VRActionHandle_t* pHandle) = 0;
	virtual EVRInputError GetInputSourceHandle(const char* pchInputSourcePath, VRInputValueHandle_t* pHandle) = 0;
	virtual EVRInputError UpdateActionState(VR_ARRAY_COUNT(unSetCount) VRActiveActionSet_t* pSets, uint32_t unSizeOfVRSelectedActionSet_t, uint32_t unSetCount) = 0;
	virtual EVRInputError GetDigitalActionData(VRActionHandle_t action, InputDigitalActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetAnalogActionData(VRActionHandle_t action, InputAnalogActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetPoseActionDataRelativeToNow(VRActionHandle_t action, ETrackingUniverseOrigin eOrigin, float fPredictedSecondsFromNow, InputPoseActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetPoseActionDataForNextFrame(VRActionHandle_t action, ETrackingUniverseOrigin eOrigin, InputPoseActionData_t* pActionData, uint32_t unActionDataSize, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetSkeletalActionData(VRActionHandle_t action, InputSkeletalActionData_t* pActionData, uint32_t unActionDataSize) = 0;
	virtual EVRInputError GetDominantHand(ETrackedControllerRole* peDominantHand) = 0;
	virtual EVRInputError SetDominantHand(ETrackedControllerRole eDominantHand) = 0;
	virtual EVRInputError GetEyeTrackingDataRelativeToNow(VRActionHandle_t action, vr::ETrackingUniverseOrigin eOrigin, float fPredictedSecondsFromNow, vr::VREyeTrackingData_t* pEyeTrackingData, uint32_t ulEyeTrackingDataSize) = 0;
	virtual EVRInputError GetEyeTrackingDataForNextFrame(VRActionHandle_t action, vr::ETrackingUniverseOrigin eOrigin, vr::VREyeTrackingData_t* pEyeTrackingData, uint32_t ulEyeTrackingDataSize) = 0;
	virtual EVRInputError GetBoneCount(VRActionHandle_t action, uint32_t* pBoneCount) = 0;
	virtual EVRInputError GetBoneHierarchy(VRActionHandle_t action, VR_ARRAY_COUNT(unIndexArayCount) BoneIndex_t* pParentIndices, uint32_t unIndexArayCount) = 0;
	virtual EVRInputError GetBoneName(VRActionHandle_t action, BoneIndex_t nBoneIndex, VR_OUT_STRING() char* pchBoneName, uint32_t unNameBufferSize) = 0;
	virtual EVRInputError GetSkeletalReferenceTransforms(VRActionHandle_t action, EVRSkeletalTransformSpace eTransformSpace, EVRSkeletalReferencePose eReferencePose, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError GetSkeletalTrackingLevel(VRActionHandle_t action, EVRSkeletalTrackingLevel* pSkeletalTrackingLevel) = 0;
	virtual EVRInputError GetSkeletalBoneData(VRActionHandle_t action, EVRSkeletalTransformSpace eTransformSpace, EVRSkeletalMotionRange eMotionRange, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError GetSkeletalSummaryData(VRActionHandle_t action, EVRSummaryType eSummaryType, VRSkeletalSummaryData_t* pSkeletalSummaryData) = 0;
	virtual EVRInputError GetSkeletalBoneDataCompressed(VRActionHandle_t action, EVRSkeletalMotionRange eMotionRange, VR_OUT_BUFFER_COUNT(unCompressedSize) void* pvCompressedData, uint32_t unCompressedSize, uint32_t* punRequiredCompressedSize) = 0;
	virtual EVRInputError DecompressSkeletalBoneData(const void* pvCompressedBuffer, uint32_t unCompressedBufferSize, EVRSkeletalTransformSpace eTransformSpace, VR_ARRAY_COUNT(unTransformArrayCount) VRBoneTransform_t* pTransformArray, uint32_t unTransformArrayCount) = 0;
	virtual EVRInputError TriggerHapticVibrationAction(VRActionHandle_t action, float fStartSecondsFromNow, float fDurationSeconds, float fFrequency, float fAmplitude, VRInputValueHandle_t ulRestrictToDevice) = 0;
	virtual EVRInputError GetActionOrigins(VRActionSetHandle_t actionSetHandle, VRActionHandle_t digitalActionHandle, VR_ARRAY_COUNT(originOutCount) VRInputValueHandle_t* originsOut, uint32_t originOutCount) = 0;
	virtual EVRInputError GetOriginLocalizedName(VRInputValueHandle_t origin, VR_OUT_STRING() char* pchNameArray, uint32_t unNameArraySize, int32_t unStringSectionsToInclude) = 0;
	virtual EVRInputError GetOriginTrackedDeviceInfo(VRInputValueHandle_t origin, InputOriginInfo_t* pOriginInfo, uint32_t unOriginInfoSize) = 0;
	virtual EVRInputError GetActionBindingInfo(VRActionHandle_t action, VR_ARRAY_COUNT(unBindingInfoCount) InputBindingInfo_t* pOriginInfo, uint32_t unBindingInfoSize, uint32_t unBindingInfoCount, uint32_t* punReturnedBindingInfoCount) = 0;
	virtual EVRInputError ShowActionOrigins(VRActionSetHandle_t actionSetHandle, VRActionHandle_t ulActionHandle) = 0;
	virtual EVRInputError ShowBindingsForActionSet(VR_ARRAY_COUNT(unSetCount) VRActiveActionSet_t* pSets, uint32_t unSizeOfVRSelectedActionSet_t, uint32_t unSetCount, VRInputValueHandle_t originToHighlight) = 0;
	virtual EVRInputError GetComponentStateForBinding(const char* pchRenderModelName, const char* pchComponentName, const InputBindingInfo_t* pOriginInfo, uint32_t unBindingInfoSize, uint32_t unBindingInfoCount, vr::RenderModel_ComponentState_t* pComponentState) = 0;
	virtual bool IsUsingLegacyInput() = 0;
	virtual EVRInputError OpenBindingUI(const char* pchAppKey, VRActionSetHandle_t ulActionSetHandle, VRInputValueHandle_t ulDeviceHandle, bool bShowOnDesktop) = 0;
	virtual EVRInputError GetBindingVariant(vr::VRInputValueHandle_t ulDevicePath, VR_OUT_STRING() char* pchVariantArray, uint32_t unVariantArraySize) = 0;
};


static const char* const IVRIOBuffer_001_Version = "IVRIOBuffer_001";

class IVRIOBuffer_001
{
public:
	virtual vr::EIOBufferError Open(const char* pchPath, vr::EIOBufferMode mode, uint32_t unElementSize, uint32_t unElements, vr::IOBufferHandle_t* pulBuffer) = 0;
	virtual vr::EIOBufferError Close(vr::IOBufferHandle_t ulBuffer) = 0;
	virtual vr::EIOBufferError Read(vr::IOBufferHandle_t ulBuffer, void* pDst, uint32_t unBytes, uint32_t* punRead) = 0;
	virtual vr::EIOBufferError Write(vr::IOBufferHandle_t ulBuffer, void* pSrc, uint32_t unBytes) = 0;
	virtual vr::PropertyContainerHandle_t PropertyContainer(vr::IOBufferHandle_t ulBuffer) = 0;
};


static const char* const IVRIOBuffer_002_Version = "IVRIOBuffer_002";

class IVRIOBuffer_002
{
public:
	virtual vr::EIOBufferError Open(const char* pchPath, vr::EIOBufferMode mode, uint32_t unElementSize, uint32_t unElements, vr::IOBufferHandle_t* pulBuffer) = 0;
	virtual vr::EIOBufferError Close(vr::IOBufferHandle_t ulBuffer) = 0;
	virtual vr::EIOBufferError Read(vr::IOBufferHandle_t ulBuffer, void* pDst, uint32_t unBytes, uint32_t* punRead) = 0;
	virtual vr::EIOBufferError Write(vr::IOBufferHandle_t ulBuffer, void* pSrc, uint32_t unBytes) = 0;
	virtual vr::PropertyContainerHandle_t PropertyContainer(vr::IOBufferHandle_t ulBuffer) = 0;
	virtual bool HasReaders(vr::IOBufferHandle_t ulBuffer) = 0;
};


static const char* const IVRIPCResourceManagerClient_001_Version = "IVRIPCResourceManagerClient_001";

class IVRIPCResourceManagerClient_001
{
public:
	virtual bool NewSharedVulkanImage(uint32_t nImageFormat, uint32_t nWidth, uint32_t nHeight, bool bRenderable, bool bMappable, bool bComputeAccess, uint32_t unMipLevels, uint32_t unArrayLayerCount, vr::SharedTextureHandle_t* pSharedHandle) = 0;
	virtual bool NewSharedVulkanBuffer(uint32_t nSize, uint32_t nUsageFlags, vr::SharedTextureHandle_t* pSharedHandle) = 0;
	virtual bool NewSharedVulkanSemaphore(vr::SharedTextureHandle_t* pSharedHandle) = 0;
	virtual bool RefResource(vr::SharedTextureHandle_t hSharedHandle, uint64_t* pNewIpcHandle) = 0;
	virtual bool UnrefResource(vr::SharedTextureHandle_t hSharedHandle) = 0;
	virtual bool GetDmabufFormats(uint32_t* pOutFormatCount, uint32_t* pOutFormats) = 0;
	virtual bool GetDmabufModifiers(vr::EVRApplicationType eApplicationType, uint32_t unDRMFormat, uint32_t* pOutModifierCount, uint64_t* pOutModifiers) = 0;
	virtual bool ImportDmabuf(vr::EVRApplicationType eApplicationType, vr::DmabufAttributes_t* pDmabufAttributes, vr::SharedTextureHandle_t* pSharedHandle) = 0;
	virtual bool ReceiveSharedFd(uint64_t ulIpcHandle, int* pOutFd) = 0;

protected:
	virtual ~IVRIPCResourceManagerClient_001() {};
};


static const char* const IVRIPCResourceManagerClient_002_Version = "IVRIPCResourceManagerClient_002";

class IVRIPCResourceManagerClient_002
{
public:
	virtual bool NewSharedVulkanImage(uint32_t nImageFormat, uint32_t nWidth, uint32_t nHeight, bool bRenderable, bool bMappable, bool bComputeAccess, uint32_t unMipLevels, uint32_t unArrayLayerCount, vr::SharedTextureHandle_t* pSharedHandle) = 0;
	virtual bool NewSharedVulkanBuffer(uint32_t nSize, uint32_t nUsageFlags, vr::SharedTextureHandle_t* pSharedHandle) = 0;
	virtual bool NewSharedVulkanSemaphore(bool bCounting, vr::SharedTextureHandle_t* pSharedHandle) = 0;
	virtual bool RefResource(vr::SharedTextureHandle_t hSharedHandle, uint64_t* pNewIpcHandle) = 0;
	virtual bool UnrefResource(vr::SharedTextureHandle_t hSharedHandle) = 0;
	virtual bool GetDmabufFormats(uint32_t* pOutFormatCount, uint32_t* pOutFormats) = 0;
	virtual bool GetDmabufModifiers(vr::EVRApplicationType eApplicationType, uint32_t unDRMFormat, uint32_t* pOutModifierCount, uint64_t* pOutModifiers) = 0;
	virtual bool ImportDmabuf(vr::EVRApplicationType eApplicationType, vr::DmabufAttributes_t* pDmabufAttributes, vr::SharedTextureHandle_t* pSharedHandle) = 0;
	virtual bool ReceiveSharedFd(uint64_t ulIpcHandle, int* pOutFd) = 0;

protected:
	virtual ~IVRIPCResourceManagerClient_002() {};
};


static const char* const IVRIPCResourceManagerClient_003_Version = "IVRIPCResourceManagerClient_003";

class IVRIPCResourceManagerClient_003
{
public:
	virtual bool NewSharedVulkanImage(uint32_t nImageFormat, uint32_t nWidth, uint32_t nHeight, bool bRenderable, bool bMappable, bool bComputeAccess, uint32_t unMipLevels, uint32_t unArrayLayerCount, uint32_t unAdditionalVkCreateFlags, uint32_t unAdditionalVkUsageFlags, vr::SharedTextureHandle_t* pSharedHandle) = 0;
	virtual bool NewSharedVulkanBuffer(uint32_t nSize, uint32_t nUsageFlags, vr::SharedTextureHandle_t* pSharedHandle) = 0;
	virtual bool NewSharedVulkanSemaphore(bool bCounting, vr::SharedTextureHandle_t* pSharedHandle) = 0;
	virtual bool RefResource(vr::SharedTextureHandle_t hSharedHandle, uint64_t* pNewIpcHandle) = 0;
	virtual bool UnrefResource(vr::SharedTextureHandle_t hSharedHandle) = 0;
	virtual bool GetDmabufFormats(uint32_t* pOutFormatCount, uint32_t* pOutFormats) = 0;
	virtual bool GetDmabufModifiers(vr::EVRApplicationType eApplicationType, uint32_t unDRMFormat, uint32_t* pOutModifierCount, uint64_t* pOutModifiers) = 0;
	virtual bool ImportDmabuf(vr::EVRApplicationType eApplicationType, vr::DmabufAttributes_t* pDmabufAttributes, vr::SharedTextureHandle_t* pSharedHandle) = 0;
	virtual bool ReceiveSharedFd(uint64_t ulIpcHandle, int* pOutFd) = 0;

protected:
	virtual ~IVRIPCResourceManagerClient_003() {};
};


static const char* const IVRNotifications_001_Version = "IVRNotifications_001";

class IVRNotifications_001
{
public:
	virtual uint32_t GetErrorString(NotificationError_t error, VR_OUT_STRING() char* pchBuffer, uint32_t unBufferSize) = 0;
	VR_METHOD_DESC(Create a new notification.)
	virtual NotificationError_t CreateNotification(VROverlayHandle_t ulOverlayHandle, uint64_t ulUserValue, const char* strType, const char* strText, const char* strCategory, const NotificationBitmap* photo, vr::VRNotificationId* notificationId) = 0;
	VR_METHOD_DESC(Dismiss the notification)
	virtual NotificationError_t DismissNotification(vr::VRNotificationId notificationId) = 0;
};


static const char* const IVRNotifications_002_Version = "IVRNotifications_002";

class IVRNotifications_002
{
public:
	virtual EVRNotificationError CreateNotification(VROverlayHandle_t ulOverlayHandle, uint64_t ulUserValue, EVRNotificationType type, const char* pchText, EVRNotificationStyle style, const NotificationBitmap_t* pImage, VRNotificationId* pNotificationId) = 0;
	virtual EVRNotificationError RemoveNotification(VRNotificationId notificationId) = 0;
};


static const char* const IVROverlay_001_Version = "IVROverlay_001";

class IVROverlay_001
{
public:
	virtual VROverlayError FindOverlay(const char* pchOverlayKey, vr::VROverlayHandle_t* pOverlayHandle) = 0;
	virtual VROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, vr::VROverlayHandle_t* pOverlayHandle) = 0;
	virtual VROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual const char* GetOverlayErrorNameFromEnum(VROverlayError error) = 0;
	virtual VROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual VROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual VROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual VROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual VROverlayError SetOverlayGamma(VROverlayHandle_t ulOverlayHandle, float fGamma) = 0;
	virtual VROverlayError GetOverlayGamma(VROverlayHandle_t ulOverlayHandle, float* pfGamma) = 0;
	virtual VROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual VROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual VROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual VROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual VROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual VROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, TrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual VROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, TrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual VROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual VROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual VROverlayError GetOverlayVisibility(VROverlayHandle_t ulOverlayHandle, VROverlayVisibility* peOverlayVisibility) = 0;
	virtual VROverlayError SetOverlayVisibility(VROverlayHandle_t ulOverlayHandle, VROverlayVisibility eOverlayVisibility) = 0;
	virtual VROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent) = 0;
	virtual VROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual VROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual VROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual VROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, vr::TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual VROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void* pTexture) = 0;
	virtual VROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual VROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual bool IsSystemOverlayVisible() = 0;
	virtual bool IsActiveSystemOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetSystemOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual VROverlayError GetSystemOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
};


static const char* const IVROverlay_002_Version = "IVROverlay_002";

class IVROverlay_002
{
public:
	virtual VROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual VROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual VROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual const char* GetOverlayErrorNameFromEnum(VROverlayError error) = 0;
	virtual VROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual VROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual VROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual VROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual VROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual VROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual VROverlayError SetOverlayGamma(VROverlayHandle_t ulOverlayHandle, float fGamma) = 0;
	virtual VROverlayError GetOverlayGamma(VROverlayHandle_t ulOverlayHandle, float* pfGamma) = 0;
	virtual VROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual VROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual VROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual VROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual VROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual VROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, TrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual VROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, TrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual VROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual VROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual VROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent) = 0;
	virtual VROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual VROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual VROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual VROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual VROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, GraphicsAPIConvention eTextureType, void* pTexture) = 0;
	virtual VROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual VROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual VROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual VROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
};


static const char* const IVROverlay_003_Version = "IVROverlay_003";

class IVROverlay_003
{
public:
	virtual VROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual VROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual VROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, VROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, VROverlayError* pError = 0L) = 0;
	virtual VROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(VROverlayError error) = 0;
	virtual VROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual VROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual VROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual VROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual VROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual VROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual VROverlayError SetOverlayGamma(VROverlayHandle_t ulOverlayHandle, float fGamma) = 0;
	virtual VROverlayError GetOverlayGamma(VROverlayHandle_t ulOverlayHandle, float* pfGamma) = 0;
	virtual VROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual VROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual VROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual VROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual VROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual VROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, TrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual VROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, TrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual VROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual VROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual VROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent) = 0;
	virtual VROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual VROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual VROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual VROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual VROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, GraphicsAPIConvention eTextureType, void* pTexture) = 0;
	virtual VROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual VROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual VROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual VROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
};


static const char* const IVROverlay_004_Version = "IVROverlay_004";

class IVROverlay_004
{
public:
	virtual VROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual VROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual VROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, VROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, VROverlayError* pError = 0L) = 0;
	virtual VROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(VROverlayError error) = 0;
	virtual VROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual VROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual VROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual VROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual VROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual VROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual VROverlayError SetOverlayGamma(VROverlayHandle_t ulOverlayHandle, float fGamma) = 0;
	virtual VROverlayError GetOverlayGamma(VROverlayHandle_t ulOverlayHandle, float* pfGamma) = 0;
	virtual VROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual VROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual VROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual VROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual VROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual VROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual VROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual VROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, TrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual VROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, TrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual VROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual VROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual VROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent) = 0;
	virtual VROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual VROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual VROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual VROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual VROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, GraphicsAPIConvention eTextureType, void* pTexture) = 0;
	virtual VROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual VROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual VROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual VROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
};


static const char* const IVROverlay_005_Version = "IVROverlay_005";

class IVROverlay_005
{
public:
	virtual VROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual VROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual VROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, VROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, VROverlayError* pError = 0L) = 0;
	virtual VROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(VROverlayError error) = 0;
	virtual VROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual VROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual VROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual VROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual VROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual VROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual VROverlayError SetOverlayGamma(VROverlayHandle_t ulOverlayHandle, float fGamma) = 0;
	virtual VROverlayError GetOverlayGamma(VROverlayHandle_t ulOverlayHandle, float* pfGamma) = 0;
	virtual VROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual VROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual VROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual VROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual VROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual VROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual VROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual VROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, TrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual VROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, TrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual VROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual VROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual VROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent) = 0;
	virtual VROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual VROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual VROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual VROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual bool IsFocusOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, GraphicsAPIConvention eTextureType, void* pTexture) = 0;
	virtual VROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual VROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual VROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual VROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual VROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
};


static const char* const IVROverlay_006_Version = "IVROverlay_006";

class IVROverlay_006
{
public:
	virtual VROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual VROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual VROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, VROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, VROverlayError* pError = 0L) = 0;
	virtual VROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(VROverlayError error) = 0;
	virtual VROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual VROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual VROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual VROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual VROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual VROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual VROverlayError SetOverlayGamma(VROverlayHandle_t ulOverlayHandle, float fGamma) = 0;
	virtual VROverlayError GetOverlayGamma(VROverlayHandle_t ulOverlayHandle, float* pfGamma) = 0;
	virtual VROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual VROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual VROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual VROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual VROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual VROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual VROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual VROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, TrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual VROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, TrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual VROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual VROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual VROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent) = 0;
	virtual VROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual VROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual VROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual VROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetGamepadFocusOverlay() = 0;
	virtual VROverlayError SetGamepadFocusOverlay(VROverlayHandle_t ulNewFocusOverlay) = 0;
	virtual VROverlayError SetOverlayNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom, VROverlayHandle_t ulTo) = 0;
	virtual VROverlayError MoveGamepadFocusToNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom) = 0;
	virtual VROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, GraphicsAPIConvention eTextureType, void* pTexture) = 0;
	virtual VROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual VROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual VROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual VROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual VROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual VROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual VROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
};


static const char* const IVROverlay_007_Version = "IVROverlay_007";

class IVROverlay_007
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual EVROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetGamepadFocusOverlay() = 0;
	virtual EVROverlayError SetGamepadFocusOverlay(VROverlayHandle_t ulNewFocusOverlay) = 0;
	virtual EVROverlayError SetOverlayNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom, VROverlayHandle_t ulTo) = 0;
	virtual EVROverlayError MoveGamepadFocusToNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
};


static const char* const IVROverlay_008_Version = "IVROverlay_008";

class IVROverlay_008
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual EVROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetGamepadFocusOverlay() = 0;
	virtual EVROverlayError SetGamepadFocusOverlay(VROverlayHandle_t ulNewFocusOverlay) = 0;
	virtual EVROverlayError SetOverlayNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom, VROverlayHandle_t ulTo) = 0;
	virtual EVROverlayError MoveGamepadFocusToNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
};


static const char* const IVROverlay_010_Version = "IVROverlay_010";

class IVROverlay_010
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual EVROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetGamepadFocusOverlay() = 0;
	virtual EVROverlayError SetGamepadFocusOverlay(VROverlayHandle_t ulNewFocusOverlay) = 0;
	virtual EVROverlayError SetOverlayNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom, VROverlayHandle_t ulTo) = 0;
	virtual EVROverlayError MoveGamepadFocusToNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
};


static const char* const IVROverlay_011_Version = "IVROverlay_011";

class IVROverlay_011
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual EVROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetGamepadFocusOverlay() = 0;
	virtual EVROverlayError SetGamepadFocusOverlay(VROverlayHandle_t ulNewFocusOverlay) = 0;
	virtual EVROverlayError SetOverlayNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom, VROverlayHandle_t ulTo) = 0;
	virtual EVROverlayError MoveGamepadFocusToNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, EGraphicsAPIConvention* pAPI, EColorSpace* pColorSpace) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
};


static const char* const IVROverlay_012_Version = "IVROverlay_012";

class IVROverlay_012
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual EVROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetGamepadFocusOverlay() = 0;
	virtual EVROverlayError SetGamepadFocusOverlay(VROverlayHandle_t ulNewFocusOverlay) = 0;
	virtual EVROverlayError SetOverlayNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom, VROverlayHandle_t ulTo) = 0;
	virtual EVROverlayError MoveGamepadFocusToNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, EGraphicsAPIConvention* pAPI, EColorSpace* pColorSpace) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
};


static const char* const IVROverlay_013_Version = "IVROverlay_013";

class IVROverlay_013
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float fTexelAspect) = 0;
	virtual EVROverlayError GetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float* pfTexelAspect) = 0;
	virtual EVROverlayError SetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t unSortOrder) = 0;
	virtual EVROverlayError GetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t* punSortOrder) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual EVROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetGamepadFocusOverlay() = 0;
	virtual EVROverlayError SetGamepadFocusOverlay(VROverlayHandle_t ulNewFocusOverlay) = 0;
	virtual EVROverlayError SetOverlayNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom, VROverlayHandle_t ulTo) = 0;
	virtual EVROverlayError MoveGamepadFocusToNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, EGraphicsAPIConvention* pAPI, EColorSpace* pColorSpace) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
	virtual EVROverlayError SetOverlayIntersectionMask(VROverlayHandle_t ulOverlayHandle, VROverlayIntersectionMaskPrimitive_t* pMaskPrimitives, uint32_t unNumMaskPrimitives, uint32_t unPrimitiveSize = sizeof(VROverlayIntersectionMaskPrimitive_t)) = 0;
};


static const char* const IVROverlay_014_Version = "IVROverlay_014";

class IVROverlay_014
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float fTexelAspect) = 0;
	virtual EVROverlayError GetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float* pfTexelAspect) = 0;
	virtual EVROverlayError SetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t unSortOrder) = 0;
	virtual EVROverlayError GetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t* punSortOrder) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual EVROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetGamepadFocusOverlay() = 0;
	virtual EVROverlayError SetGamepadFocusOverlay(VROverlayHandle_t ulNewFocusOverlay) = 0;
	virtual EVROverlayError SetOverlayNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom, VROverlayHandle_t ulTo) = 0;
	virtual EVROverlayError MoveGamepadFocusToNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, ETextureType* pAPIType, EColorSpace* pColorSpace, VRTextureBounds_t* pTextureBounds) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
	virtual EVROverlayError SetOverlayIntersectionMask(VROverlayHandle_t ulOverlayHandle, VROverlayIntersectionMaskPrimitive_t* pMaskPrimitives, uint32_t unNumMaskPrimitives, uint32_t unPrimitiveSize = sizeof(VROverlayIntersectionMaskPrimitive_t)) = 0;
	virtual EVROverlayError GetOverlayFlags(VROverlayHandle_t ulOverlayHandle, uint32_t* pFlags) = 0;
	virtual VRMessageOverlayResponse ShowMessageOverlay(const char* pchText, const char* pchCaption, const char* pchButton0Text, const char* pchButton1Text = nullptr, const char* pchButton2Text = nullptr, const char* pchButton3Text = nullptr) = 0;
};


static const char* const IVROverlay_016_Version = "IVROverlay_016";

class IVROverlay_016
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError SetOverlayName(VROverlayHandle_t ulOverlayHandle, const char* pchName) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float fTexelAspect) = 0;
	virtual EVROverlayError GetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float* pfTexelAspect) = 0;
	virtual EVROverlayError SetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t unSortOrder) = 0;
	virtual EVROverlayError GetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t* punSortOrder) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual EVROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual uint32_t GetOverlayRenderModel(vr::VROverlayHandle_t ulOverlayHandle, char* pchValue, uint32_t unBufferSize, HmdColor_t* pColor, vr::EVROverlayError* pError) = 0;
	virtual vr::EVROverlayError SetOverlayRenderModel(vr::VROverlayHandle_t ulOverlayHandle, const char* pchRenderModel, const HmdColor_t* pColor) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual vr::EVROverlayError GetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t* ulOverlayHandleParent, HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual vr::EVROverlayError SetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulOverlayHandleParent, const HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetGamepadFocusOverlay() = 0;
	virtual EVROverlayError SetGamepadFocusOverlay(VROverlayHandle_t ulNewFocusOverlay) = 0;
	virtual EVROverlayError SetOverlayNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom, VROverlayHandle_t ulTo) = 0;
	virtual EVROverlayError MoveGamepadFocusToNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, ETextureType* pAPIType, EColorSpace* pColorSpace, VRTextureBounds_t* pTextureBounds) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
	virtual EVROverlayError SetOverlayIntersectionMask(VROverlayHandle_t ulOverlayHandle, VROverlayIntersectionMaskPrimitive_t* pMaskPrimitives, uint32_t unNumMaskPrimitives, uint32_t unPrimitiveSize = sizeof(VROverlayIntersectionMaskPrimitive_t)) = 0;
	virtual EVROverlayError GetOverlayFlags(VROverlayHandle_t ulOverlayHandle, uint32_t* pFlags) = 0;
	virtual VRMessageOverlayResponse ShowMessageOverlay(const char* pchText, const char* pchCaption, const char* pchButton0Text, const char* pchButton1Text = nullptr, const char* pchButton2Text = nullptr, const char* pchButton3Text = nullptr) = 0;
	virtual void CloseMessageOverlay() = 0;
};


static const char* const IVROverlay_017_Version = "IVROverlay_017";

class IVROverlay_017
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError SetOverlayName(VROverlayHandle_t ulOverlayHandle, const char* pchName) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float fTexelAspect) = 0;
	virtual EVROverlayError GetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float* pfTexelAspect) = 0;
	virtual EVROverlayError SetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t unSortOrder) = 0;
	virtual EVROverlayError GetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t* punSortOrder) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual EVROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual uint32_t GetOverlayRenderModel(vr::VROverlayHandle_t ulOverlayHandle, char* pchValue, uint32_t unBufferSize, HmdColor_t* pColor, vr::EVROverlayError* pError) = 0;
	virtual vr::EVROverlayError SetOverlayRenderModel(vr::VROverlayHandle_t ulOverlayHandle, const char* pchRenderModel, const HmdColor_t* pColor) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual vr::EVROverlayError GetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t* ulOverlayHandleParent, HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual vr::EVROverlayError SetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulOverlayHandleParent, const HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unControllerDeviceIndex) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetGamepadFocusOverlay() = 0;
	virtual EVROverlayError SetGamepadFocusOverlay(VROverlayHandle_t ulNewFocusOverlay) = 0;
	virtual EVROverlayError SetOverlayNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom, VROverlayHandle_t ulTo) = 0;
	virtual EVROverlayError MoveGamepadFocusToNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom) = 0;
	virtual EVROverlayError SetOverlayDualAnalogTransform(VROverlayHandle_t ulOverlay, EDualAnalogWhich eWhich, const HmdVector2_t& vCenter, float fRadius) = 0;
	virtual EVROverlayError GetOverlayDualAnalogTransform(VROverlayHandle_t ulOverlay, EDualAnalogWhich eWhich, HmdVector2_t* pvCenter, float* pfRadius) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, ETextureType* pAPIType, EColorSpace* pColorSpace, VRTextureBounds_t* pTextureBounds) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
	virtual EVROverlayError SetOverlayIntersectionMask(VROverlayHandle_t ulOverlayHandle, VROverlayIntersectionMaskPrimitive_t* pMaskPrimitives, uint32_t unNumMaskPrimitives, uint32_t unPrimitiveSize = sizeof(VROverlayIntersectionMaskPrimitive_t)) = 0;
	virtual EVROverlayError GetOverlayFlags(VROverlayHandle_t ulOverlayHandle, uint32_t* pFlags) = 0;
	virtual VRMessageOverlayResponse ShowMessageOverlay(const char* pchText, const char* pchCaption, const char* pchButton0Text, const char* pchButton1Text = nullptr, const char* pchButton2Text = nullptr, const char* pchButton3Text = nullptr) = 0;
	virtual void CloseMessageOverlay() = 0;
};


static const char* const IVROverlay_018_Version = "IVROverlay_018";

class IVROverlay_018
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError SetOverlayName(VROverlayHandle_t ulOverlayHandle, const char* pchName) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float fTexelAspect) = 0;
	virtual EVROverlayError GetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float* pfTexelAspect) = 0;
	virtual EVROverlayError SetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t unSortOrder) = 0;
	virtual EVROverlayError GetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t* punSortOrder) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual EVROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual uint32_t GetOverlayRenderModel(vr::VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, HmdColor_t* pColor, vr::EVROverlayError* pError) = 0;
	virtual vr::EVROverlayError SetOverlayRenderModel(vr::VROverlayHandle_t ulOverlayHandle, const char* pchRenderModel, const HmdColor_t* pColor) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual vr::EVROverlayError GetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t* ulOverlayHandleParent, HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual vr::EVROverlayError SetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulOverlayHandleParent, const HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetGamepadFocusOverlay() = 0;
	virtual EVROverlayError SetGamepadFocusOverlay(VROverlayHandle_t ulNewFocusOverlay) = 0;
	virtual EVROverlayError SetOverlayNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom, VROverlayHandle_t ulTo) = 0;
	virtual EVROverlayError MoveGamepadFocusToNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom) = 0;
	virtual EVROverlayError SetOverlayDualAnalogTransform(VROverlayHandle_t ulOverlay, EDualAnalogWhich eWhich, const HmdVector2_t& vCenter, float fRadius) = 0;
	virtual EVROverlayError GetOverlayDualAnalogTransform(VROverlayHandle_t ulOverlay, EDualAnalogWhich eWhich, HmdVector2_t* pvCenter, float* pfRadius) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, ETextureType* pAPIType, EColorSpace* pColorSpace, VRTextureBounds_t* pTextureBounds) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
	virtual EVROverlayError SetOverlayIntersectionMask(VROverlayHandle_t ulOverlayHandle, VROverlayIntersectionMaskPrimitive_t* pMaskPrimitives, uint32_t unNumMaskPrimitives, uint32_t unPrimitiveSize = sizeof(VROverlayIntersectionMaskPrimitive_t)) = 0;
	virtual EVROverlayError GetOverlayFlags(VROverlayHandle_t ulOverlayHandle, uint32_t* pFlags) = 0;
	virtual VRMessageOverlayResponse ShowMessageOverlay(const char* pchText, const char* pchCaption, const char* pchButton0Text, const char* pchButton1Text = nullptr, const char* pchButton2Text = nullptr, const char* pchButton3Text = nullptr) = 0;
	virtual void CloseMessageOverlay() = 0;
};


static const char* const IVROverlay_019_Version = "IVROverlay_019";

class IVROverlay_019
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetHighQualityOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetHighQualityOverlay() = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError SetOverlayName(VROverlayHandle_t ulOverlayHandle, const char* pchName) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float fTexelAspect) = 0;
	virtual EVROverlayError GetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float* pfTexelAspect) = 0;
	virtual EVROverlayError SetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t unSortOrder) = 0;
	virtual EVROverlayError GetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t* punSortOrder) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual EVROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual uint32_t GetOverlayRenderModel(vr::VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, HmdColor_t* pColor, vr::EVROverlayError* pError) = 0;
	virtual vr::EVROverlayError SetOverlayRenderModel(vr::VROverlayHandle_t ulOverlayHandle, const char* pchRenderModel, const HmdColor_t* pColor) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual vr::EVROverlayError GetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t* ulOverlayHandleParent, HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual vr::EVROverlayError SetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulOverlayHandleParent, const HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetGamepadFocusOverlay() = 0;
	virtual EVROverlayError SetGamepadFocusOverlay(VROverlayHandle_t ulNewFocusOverlay) = 0;
	virtual EVROverlayError SetOverlayNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom, VROverlayHandle_t ulTo) = 0;
	virtual EVROverlayError MoveGamepadFocusToNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom) = 0;
	virtual EVROverlayError SetOverlayDualAnalogTransform(VROverlayHandle_t ulOverlay, EDualAnalogWhich eWhich, const HmdVector2_t* pvCenter, float fRadius) = 0;
	virtual EVROverlayError GetOverlayDualAnalogTransform(VROverlayHandle_t ulOverlay, EDualAnalogWhich eWhich, HmdVector2_t* pvCenter, float* pfRadius) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, ETextureType* pAPIType, EColorSpace* pColorSpace, VRTextureBounds_t* pTextureBounds) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
	virtual EVROverlayError SetOverlayIntersectionMask(VROverlayHandle_t ulOverlayHandle, VROverlayIntersectionMaskPrimitive_t* pMaskPrimitives, uint32_t unNumMaskPrimitives, uint32_t unPrimitiveSize = sizeof(VROverlayIntersectionMaskPrimitive_t)) = 0;
	virtual EVROverlayError GetOverlayFlags(VROverlayHandle_t ulOverlayHandle, uint32_t* pFlags) = 0;
	virtual VRMessageOverlayResponse ShowMessageOverlay(const char* pchText, const char* pchCaption, const char* pchButton0Text, const char* pchButton1Text = nullptr, const char* pchButton2Text = nullptr, const char* pchButton3Text = nullptr) = 0;
	virtual void CloseMessageOverlay() = 0;
};


static const char* const IVROverlay_020_Version = "IVROverlay_020";

class IVROverlay_020
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError SetOverlayName(VROverlayHandle_t ulOverlayHandle, const char* pchName) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float fTexelAspect) = 0;
	virtual EVROverlayError GetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float* pfTexelAspect) = 0;
	virtual EVROverlayError SetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t unSortOrder) = 0;
	virtual EVROverlayError GetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t* punSortOrder) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float fMinDistanceInMeters, float fMaxDistanceInMeters) = 0;
	virtual EVROverlayError GetOverlayAutoCurveDistanceRangeInMeters(VROverlayHandle_t ulOverlayHandle, float* pfMinDistanceInMeters, float* pfMaxDistanceInMeters) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual uint32_t GetOverlayRenderModel(vr::VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, HmdColor_t* pColor, vr::EVROverlayError* pError) = 0;
	virtual vr::EVROverlayError SetOverlayRenderModel(vr::VROverlayHandle_t ulOverlayHandle, const char* pchRenderModel, const HmdColor_t* pColor) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual vr::EVROverlayError GetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t* ulOverlayHandleParent, HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual vr::EVROverlayError SetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulOverlayHandleParent, const HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual vr::VROverlayHandle_t GetGamepadFocusOverlay() = 0;
	virtual EVROverlayError SetGamepadFocusOverlay(VROverlayHandle_t ulNewFocusOverlay) = 0;
	virtual EVROverlayError SetOverlayNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom, VROverlayHandle_t ulTo) = 0;
	virtual EVROverlayError MoveGamepadFocusToNeighbor(EOverlayDirection eDirection, VROverlayHandle_t ulFrom) = 0;
	virtual EVROverlayError SetOverlayDualAnalogTransform(VROverlayHandle_t ulOverlay, EDualAnalogWhich eWhich, const HmdVector2_t* pvCenter, float fRadius) = 0;
	virtual EVROverlayError GetOverlayDualAnalogTransform(VROverlayHandle_t ulOverlay, EDualAnalogWhich eWhich, HmdVector2_t* pvCenter, float* pfRadius) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, ETextureType* pAPIType, EColorSpace* pColorSpace, VRTextureBounds_t* pTextureBounds) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
	virtual EVROverlayError SetOverlayIntersectionMask(VROverlayHandle_t ulOverlayHandle, VROverlayIntersectionMaskPrimitive_t* pMaskPrimitives, uint32_t unNumMaskPrimitives, uint32_t unPrimitiveSize = sizeof(VROverlayIntersectionMaskPrimitive_t)) = 0;
	virtual EVROverlayError GetOverlayFlags(VROverlayHandle_t ulOverlayHandle, uint32_t* pFlags) = 0;
	virtual VRMessageOverlayResponse ShowMessageOverlay(const char* pchText, const char* pchCaption, const char* pchButton0Text, const char* pchButton1Text = nullptr, const char* pchButton2Text = nullptr, const char* pchButton3Text = nullptr) = 0;
	virtual void CloseMessageOverlay() = 0;
};


static const char* const IVROverlay_021_Version = "IVROverlay_021";

class IVROverlay_021
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError SetOverlayName(VROverlayHandle_t ulOverlayHandle, const char* pchName) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float fTexelAspect) = 0;
	virtual EVROverlayError GetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float* pfTexelAspect) = 0;
	virtual EVROverlayError SetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t unSortOrder) = 0;
	virtual EVROverlayError GetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t* punSortOrder) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayCurvature(VROverlayHandle_t ulOverlayHandle, float fCurvature) = 0;
	virtual EVROverlayError GetOverlayCurvature(VROverlayHandle_t ulOverlayHandle, float* pfCurvature) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual uint32_t GetOverlayRenderModel(vr::VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, HmdColor_t* pColor, vr::EVROverlayError* pError) = 0;
	virtual vr::EVROverlayError SetOverlayRenderModel(vr::VROverlayHandle_t ulOverlayHandle, const char* pchRenderModel, const HmdColor_t* pColor) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual vr::EVROverlayError GetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t* ulOverlayHandleParent, HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual vr::EVROverlayError SetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulOverlayHandleParent, const HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayDualAnalogTransform(VROverlayHandle_t ulOverlay, EDualAnalogWhich eWhich, const HmdVector2_t* pvCenter, float fRadius) = 0;
	virtual EVROverlayError GetOverlayDualAnalogTransform(VROverlayHandle_t ulOverlay, EDualAnalogWhich eWhich, HmdVector2_t* pvCenter, float* pfRadius) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unDepth) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, ETextureType* pAPIType, EColorSpace* pColorSpace, VRTextureBounds_t* pTextureBounds) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
	virtual EVROverlayError SetOverlayIntersectionMask(VROverlayHandle_t ulOverlayHandle, VROverlayIntersectionMaskPrimitive_t* pMaskPrimitives, uint32_t unNumMaskPrimitives, uint32_t unPrimitiveSize = sizeof(VROverlayIntersectionMaskPrimitive_t)) = 0;
	virtual EVROverlayError GetOverlayFlags(VROverlayHandle_t ulOverlayHandle, uint32_t* pFlags) = 0;
	virtual VRMessageOverlayResponse ShowMessageOverlay(const char* pchText, const char* pchCaption, const char* pchButton0Text, const char* pchButton1Text = nullptr, const char* pchButton2Text = nullptr, const char* pchButton3Text = nullptr) = 0;
	virtual void CloseMessageOverlay() = 0;
};


static const char* const IVROverlay_022_Version = "IVROverlay_022";

class IVROverlay_022
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError SetOverlayName(VROverlayHandle_t ulOverlayHandle, const char* pchName) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError GetOverlayFlags(VROverlayHandle_t ulOverlayHandle, uint32_t* pFlags) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float fTexelAspect) = 0;
	virtual EVROverlayError GetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float* pfTexelAspect) = 0;
	virtual EVROverlayError SetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t unSortOrder) = 0;
	virtual EVROverlayError GetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t* punSortOrder) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayCurvature(VROverlayHandle_t ulOverlayHandle, float fCurvature) = 0;
	virtual EVROverlayError GetOverlayCurvature(VROverlayHandle_t ulOverlayHandle, float* pfCurvature) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual uint32_t GetOverlayRenderModel(vr::VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, HmdColor_t* pColor, vr::EVROverlayError* pError) = 0;
	virtual vr::EVROverlayError SetOverlayRenderModel(vr::VROverlayHandle_t ulOverlayHandle, const char* pchRenderModel, const HmdColor_t* pColor) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual vr::EVROverlayError GetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t* ulOverlayHandleParent, HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual vr::EVROverlayError SetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulOverlayHandleParent, const HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformCursor(VROverlayHandle_t ulCursorOverlayHandle, const HmdVector2_t* pvHotspot) = 0;
	virtual vr::EVROverlayError GetOverlayTransformCursor(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvHotspot) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayDualAnalogTransform(VROverlayHandle_t ulOverlay, EDualAnalogWhich eWhich, const HmdVector2_t* pvCenter, float fRadius) = 0;
	virtual EVROverlayError GetOverlayDualAnalogTransform(VROverlayHandle_t ulOverlay, EDualAnalogWhich eWhich, HmdVector2_t* pvCenter, float* pfRadius) = 0;
	virtual EVROverlayError SetOverlayIntersectionMask(VROverlayHandle_t ulOverlayHandle, VROverlayIntersectionMaskPrimitive_t* pMaskPrimitives, uint32_t unNumMaskPrimitives, uint32_t unPrimitiveSize = sizeof(VROverlayIntersectionMaskPrimitive_t)) = 0;
	virtual EVROverlayError TriggerLaserMouseHapticVibration(VROverlayHandle_t ulOverlayHandle, float fDurationSeconds, float fFrequency, float fAmplitude) = 0;
	virtual EVROverlayError SetOverlayCursor(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulCursorHandle) = 0;
	virtual EVROverlayError SetOverlayCursorPositionOverride(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvCursor) = 0;
	virtual EVROverlayError ClearOverlayCursorPositionOverride(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unBytesPerPixel) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, ETextureType* pAPIType, EColorSpace* pColorSpace, VRTextureBounds_t* pTextureBounds) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, bool bUseMinimalMode, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
	virtual VRMessageOverlayResponse ShowMessageOverlay(const char* pchText, const char* pchCaption, const char* pchButton0Text, const char* pchButton1Text = nullptr, const char* pchButton2Text = nullptr, const char* pchButton3Text = nullptr) = 0;
	virtual void CloseMessageOverlay() = 0;
};


static const char* const IVROverlay_024_Version = "IVROverlay_024";

class IVROverlay_024
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError SetOverlayName(VROverlayHandle_t ulOverlayHandle, const char* pchName) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError GetOverlayFlags(VROverlayHandle_t ulOverlayHandle, uint32_t* pFlags) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float fTexelAspect) = 0;
	virtual EVROverlayError GetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float* pfTexelAspect) = 0;
	virtual EVROverlayError SetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t unSortOrder) = 0;
	virtual EVROverlayError GetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t* punSortOrder) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayCurvature(VROverlayHandle_t ulOverlayHandle, float fCurvature) = 0;
	virtual EVROverlayError GetOverlayCurvature(VROverlayHandle_t ulOverlayHandle, float* pfCurvature) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual vr::EVROverlayError GetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t* ulOverlayHandleParent, HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual vr::EVROverlayError SetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulOverlayHandleParent, const HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformCursor(VROverlayHandle_t ulCursorOverlayHandle, const HmdVector2_t* pvHotspot) = 0;
	virtual vr::EVROverlayError GetOverlayTransformCursor(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvHotspot) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayIntersectionMask(VROverlayHandle_t ulOverlayHandle, VROverlayIntersectionMaskPrimitive_t* pMaskPrimitives, uint32_t unNumMaskPrimitives, uint32_t unPrimitiveSize = sizeof(VROverlayIntersectionMaskPrimitive_t)) = 0;
	virtual EVROverlayError TriggerLaserMouseHapticVibration(VROverlayHandle_t ulOverlayHandle, float fDurationSeconds, float fFrequency, float fAmplitude) = 0;
	virtual EVROverlayError SetOverlayCursor(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulCursorHandle) = 0;
	virtual EVROverlayError SetOverlayCursorPositionOverride(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvCursor) = 0;
	virtual EVROverlayError ClearOverlayCursorPositionOverride(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unBytesPerPixel) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, ETextureType* pAPIType, EColorSpace* pColorSpace, VRTextureBounds_t* pTextureBounds) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, uint32_t unFlags, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, uint32_t unFlags, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
	virtual VRMessageOverlayResponse ShowMessageOverlay(const char* pchText, const char* pchCaption, const char* pchButton0Text, const char* pchButton1Text = nullptr, const char* pchButton2Text = nullptr, const char* pchButton3Text = nullptr) = 0;
	virtual void CloseMessageOverlay() = 0;
};


static const char* const IVROverlay_025_Version = "IVROverlay_025";

class IVROverlay_025
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError SetOverlayName(VROverlayHandle_t ulOverlayHandle, const char* pchName) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError GetOverlayFlags(VROverlayHandle_t ulOverlayHandle, uint32_t* pFlags) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float fTexelAspect) = 0;
	virtual EVROverlayError GetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float* pfTexelAspect) = 0;
	virtual EVROverlayError SetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t unSortOrder) = 0;
	virtual EVROverlayError GetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t* punSortOrder) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayCurvature(VROverlayHandle_t ulOverlayHandle, float fCurvature) = 0;
	virtual EVROverlayError GetOverlayCurvature(VROverlayHandle_t ulOverlayHandle, float* pfCurvature) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual vr::EVROverlayError GetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t* ulOverlayHandleParent, HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual vr::EVROverlayError SetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulOverlayHandleParent, const HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformCursor(VROverlayHandle_t ulCursorOverlayHandle, const HmdVector2_t* pvHotspot) = 0;
	virtual vr::EVROverlayError GetOverlayTransformCursor(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvHotspot) = 0;
	virtual vr::EVROverlayError SetOverlayTransformProjection(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform, const VROverlayProjection_t* pProjection, vr::EVREye eEye) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayIntersectionMask(VROverlayHandle_t ulOverlayHandle, VROverlayIntersectionMaskPrimitive_t* pMaskPrimitives, uint32_t unNumMaskPrimitives, uint32_t unPrimitiveSize = sizeof(VROverlayIntersectionMaskPrimitive_t)) = 0;
	virtual EVROverlayError TriggerLaserMouseHapticVibration(VROverlayHandle_t ulOverlayHandle, float fDurationSeconds, float fFrequency, float fAmplitude) = 0;
	virtual EVROverlayError SetOverlayCursor(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulCursorHandle) = 0;
	virtual EVROverlayError SetOverlayCursorPositionOverride(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvCursor) = 0;
	virtual EVROverlayError ClearOverlayCursorPositionOverride(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unBytesPerPixel) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, ETextureType* pAPIType, EColorSpace* pColorSpace, VRTextureBounds_t* pTextureBounds) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, uint32_t unFlags, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, uint32_t unFlags, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
	virtual VRMessageOverlayResponse ShowMessageOverlay(const char* pchText, const char* pchCaption, const char* pchButton0Text, const char* pchButton1Text = nullptr, const char* pchButton2Text = nullptr, const char* pchButton3Text = nullptr) = 0;
	virtual void CloseMessageOverlay() = 0;
};


static const char* const IVROverlay_026_Version = "IVROverlay_026";

class IVROverlay_026
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError SetOverlayName(VROverlayHandle_t ulOverlayHandle, const char* pchName) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError GetOverlayFlags(VROverlayHandle_t ulOverlayHandle, uint32_t* pFlags) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float fTexelAspect) = 0;
	virtual EVROverlayError GetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float* pfTexelAspect) = 0;
	virtual EVROverlayError SetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t unSortOrder) = 0;
	virtual EVROverlayError GetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t* punSortOrder) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayCurvature(VROverlayHandle_t ulOverlayHandle, float fCurvature) = 0;
	virtual EVROverlayError GetOverlayCurvature(VROverlayHandle_t ulOverlayHandle, float* pfCurvature) = 0;
	virtual EVROverlayError SetOverlayPreCurvePitch(VROverlayHandle_t ulOverlayHandle, float fRadians) = 0;
	virtual EVROverlayError GetOverlayPreCurvePitch(VROverlayHandle_t ulOverlayHandle, float* pfRadians) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual vr::EVROverlayError GetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t* ulOverlayHandleParent, HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual vr::EVROverlayError SetOverlayTransformOverlayRelative(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulOverlayHandleParent, const HmdMatrix34_t* pmatParentOverlayToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformCursor(VROverlayHandle_t ulCursorOverlayHandle, const HmdVector2_t* pvHotspot) = 0;
	virtual vr::EVROverlayError GetOverlayTransformCursor(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvHotspot) = 0;
	virtual vr::EVROverlayError SetOverlayTransformProjection(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform, const VROverlayProjection_t* pProjection, vr::EVREye eEye) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual EVROverlayError WaitFrameSync(uint32_t nTimeoutMs) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayIntersectionMask(VROverlayHandle_t ulOverlayHandle, VROverlayIntersectionMaskPrimitive_t* pMaskPrimitives, uint32_t unNumMaskPrimitives, uint32_t unPrimitiveSize = sizeof(VROverlayIntersectionMaskPrimitive_t)) = 0;
	virtual EVROverlayError TriggerLaserMouseHapticVibration(VROverlayHandle_t ulOverlayHandle, float fDurationSeconds, float fFrequency, float fAmplitude) = 0;
	virtual EVROverlayError SetOverlayCursor(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulCursorHandle) = 0;
	virtual EVROverlayError SetOverlayCursorPositionOverride(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvCursor) = 0;
	virtual EVROverlayError ClearOverlayCursorPositionOverride(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unBytesPerPixel) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, ETextureType* pAPIType, EColorSpace* pColorSpace, VRTextureBounds_t* pTextureBounds) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, uint32_t unFlags, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, uint32_t unFlags, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
	virtual VRMessageOverlayResponse ShowMessageOverlay(const char* pchText, const char* pchCaption, const char* pchButton0Text, const char* pchButton1Text = nullptr, const char* pchButton2Text = nullptr, const char* pchButton3Text = nullptr) = 0;
	virtual void CloseMessageOverlay() = 0;
};


static const char* const IVROverlay_027_Version = "IVROverlay_027";

class IVROverlay_027
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError SetOverlayName(VROverlayHandle_t ulOverlayHandle, const char* pchName) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError GetOverlayFlags(VROverlayHandle_t ulOverlayHandle, uint32_t* pFlags) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float fTexelAspect) = 0;
	virtual EVROverlayError GetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float* pfTexelAspect) = 0;
	virtual EVROverlayError SetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t unSortOrder) = 0;
	virtual EVROverlayError GetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t* punSortOrder) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayCurvature(VROverlayHandle_t ulOverlayHandle, float fCurvature) = 0;
	virtual EVROverlayError GetOverlayCurvature(VROverlayHandle_t ulOverlayHandle, float* pfCurvature) = 0;
	virtual EVROverlayError SetOverlayPreCurvePitch(VROverlayHandle_t ulOverlayHandle, float fRadians) = 0;
	virtual EVROverlayError GetOverlayPreCurvePitch(VROverlayHandle_t ulOverlayHandle, float* pfRadians) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual EVROverlayError SetOverlayTransformCursor(VROverlayHandle_t ulCursorOverlayHandle, const HmdVector2_t* pvHotspot) = 0;
	virtual vr::EVROverlayError GetOverlayTransformCursor(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvHotspot) = 0;
	virtual vr::EVROverlayError SetOverlayTransformProjection(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform, const VROverlayProjection_t* pProjection, vr::EVREye eEye) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual EVROverlayError WaitFrameSync(uint32_t nTimeoutMs) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayIntersectionMask(VROverlayHandle_t ulOverlayHandle, VROverlayIntersectionMaskPrimitive_t* pMaskPrimitives, uint32_t unNumMaskPrimitives, uint32_t unPrimitiveSize = sizeof(VROverlayIntersectionMaskPrimitive_t)) = 0;
	virtual EVROverlayError TriggerLaserMouseHapticVibration(VROverlayHandle_t ulOverlayHandle, float fDurationSeconds, float fFrequency, float fAmplitude) = 0;
	virtual EVROverlayError SetOverlayCursor(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulCursorHandle) = 0;
	virtual EVROverlayError SetOverlayCursorPositionOverride(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvCursor) = 0;
	virtual EVROverlayError ClearOverlayCursorPositionOverride(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unBytesPerPixel) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, ETextureType* pAPIType, EColorSpace* pColorSpace, VRTextureBounds_t* pTextureBounds) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, uint32_t unFlags, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, uint32_t unFlags, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
	virtual VRMessageOverlayResponse ShowMessageOverlay(const char* pchText, const char* pchCaption, const char* pchButton0Text, const char* pchButton1Text = nullptr, const char* pchButton2Text = nullptr, const char* pchButton3Text = nullptr) = 0;
	virtual void CloseMessageOverlay() = 0;
};


static const char* const IVROverlay_028_Version = "IVROverlay_028";

class IVROverlay_028
{
public:
	virtual EVROverlayError FindOverlay(const char* pchOverlayKey, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateOverlay(const char* pchOverlayKey, const char* pchOverlayName, VROverlayHandle_t* pOverlayHandle) = 0;
	virtual EVROverlayError CreateSubviewOverlay(VROverlayHandle_t parentOverlayHandle, const char* pchSubviewOverlayKey, const char* pchSubviewOverlayName, VROverlayHandle_t* pSubviewOverlayHandle) = 0;
	virtual EVROverlayError DestroyOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual uint32_t GetOverlayKey(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual uint32_t GetOverlayName(VROverlayHandle_t ulOverlayHandle, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, EVROverlayError* pError = 0L) = 0;
	virtual EVROverlayError SetOverlayName(VROverlayHandle_t ulOverlayHandle, const char* pchName) = 0;
	virtual EVROverlayError GetOverlayImageData(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unBufferSize, uint32_t* punWidth, uint32_t* punHeight) = 0;
	virtual const char* GetOverlayErrorNameFromEnum(EVROverlayError error) = 0;
	virtual EVROverlayError SetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle, uint32_t unPID) = 0;
	virtual uint32_t GetOverlayRenderingPid(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool bEnabled) = 0;
	virtual EVROverlayError GetOverlayFlag(VROverlayHandle_t ulOverlayHandle, VROverlayFlags eOverlayFlag, bool* pbEnabled) = 0;
	virtual EVROverlayError GetOverlayFlags(VROverlayHandle_t ulOverlayHandle, uint32_t* pFlags) = 0;
	virtual EVROverlayError SetOverlayColor(VROverlayHandle_t ulOverlayHandle, float fRed, float fGreen, float fBlue) = 0;
	virtual EVROverlayError GetOverlayColor(VROverlayHandle_t ulOverlayHandle, float* pfRed, float* pfGreen, float* pfBlue) = 0;
	virtual EVROverlayError SetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float fAlpha) = 0;
	virtual EVROverlayError GetOverlayAlpha(VROverlayHandle_t ulOverlayHandle, float* pfAlpha) = 0;
	virtual EVROverlayError SetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float fTexelAspect) = 0;
	virtual EVROverlayError GetOverlayTexelAspect(VROverlayHandle_t ulOverlayHandle, float* pfTexelAspect) = 0;
	virtual EVROverlayError SetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t unSortOrder) = 0;
	virtual EVROverlayError GetOverlaySortOrder(VROverlayHandle_t ulOverlayHandle, uint32_t* punSortOrder) = 0;
	virtual EVROverlayError SetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float fWidthInMeters) = 0;
	virtual EVROverlayError GetOverlayWidthInMeters(VROverlayHandle_t ulOverlayHandle, float* pfWidthInMeters) = 0;
	virtual EVROverlayError SetOverlayCurvature(VROverlayHandle_t ulOverlayHandle, float fCurvature) = 0;
	virtual EVROverlayError GetOverlayCurvature(VROverlayHandle_t ulOverlayHandle, float* pfCurvature) = 0;
	virtual EVROverlayError SetOverlayPreCurvePitch(VROverlayHandle_t ulOverlayHandle, float fRadians) = 0;
	virtual EVROverlayError GetOverlayPreCurvePitch(VROverlayHandle_t ulOverlayHandle, float* pfRadians) = 0;
	virtual EVROverlayError SetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace eTextureColorSpace) = 0;
	virtual EVROverlayError GetOverlayTextureColorSpace(VROverlayHandle_t ulOverlayHandle, EColorSpace* peTextureColorSpace) = 0;
	virtual EVROverlayError SetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, const VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTextureBounds(VROverlayHandle_t ulOverlayHandle, VRTextureBounds_t* pOverlayTextureBounds) = 0;
	virtual EVROverlayError GetOverlayTransformType(VROverlayHandle_t ulOverlayHandle, VROverlayTransformType* peTransformType) = 0;
	virtual EVROverlayError SetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformAbsolute(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin* peTrackingOrigin, HmdMatrix34_t* pmatTrackingOriginToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unTrackedDevice, const HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceRelative(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punTrackedDevice, HmdMatrix34_t* pmatTrackedDeviceToOverlayTransform) = 0;
	virtual EVROverlayError SetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t unDeviceIndex, const char* pchComponentName) = 0;
	virtual EVROverlayError GetOverlayTransformTrackedDeviceComponent(VROverlayHandle_t ulOverlayHandle, TrackedDeviceIndex_t* punDeviceIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameSize) = 0;
	virtual EVROverlayError SetOverlayTransformCursor(VROverlayHandle_t ulCursorOverlayHandle, const HmdVector2_t* pvHotspot) = 0;
	virtual vr::EVROverlayError GetOverlayTransformCursor(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvHotspot) = 0;
	virtual vr::EVROverlayError SetOverlayTransformProjection(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToOverlayTransform, const VROverlayProjection_t* pProjection, vr::EVREye eEye) = 0;
	virtual EVROverlayError SetSubviewPosition(VROverlayHandle_t ulOverlayHandle, float fX, float fY) = 0;
	virtual EVROverlayError ShowOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError HideOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual bool IsOverlayVisible(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError GetTransformForOverlayCoordinates(VROverlayHandle_t ulOverlayHandle, ETrackingUniverseOrigin eTrackingOrigin, HmdVector2_t coordinatesInOverlay, HmdMatrix34_t* pmatTransform) = 0;
	virtual EVROverlayError WaitFrameSync(uint32_t nTimeoutMs) = 0;
	virtual bool PollNextOverlayEvent(VROverlayHandle_t ulOverlayHandle, VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual EVROverlayError GetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod* peInputMethod) = 0;
	virtual EVROverlayError SetOverlayInputMethod(VROverlayHandle_t ulOverlayHandle, VROverlayInputMethod eInputMethod) = 0;
	virtual EVROverlayError GetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, HmdVector2_t* pvecMouseScale) = 0;
	virtual EVROverlayError SetOverlayMouseScale(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvecMouseScale) = 0;
	virtual bool ComputeOverlayIntersection(VROverlayHandle_t ulOverlayHandle, const VROverlayIntersectionParams_t* pParams, VROverlayIntersectionResults_t* pResults) = 0;
	virtual bool IsHoverTargetOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayIntersectionMask(VROverlayHandle_t ulOverlayHandle, VROverlayIntersectionMaskPrimitive_t* pMaskPrimitives, uint32_t unNumMaskPrimitives, uint32_t unPrimitiveSize = sizeof(VROverlayIntersectionMaskPrimitive_t)) = 0;
	virtual EVROverlayError TriggerLaserMouseHapticVibration(VROverlayHandle_t ulOverlayHandle, float fDurationSeconds, float fFrequency, float fAmplitude) = 0;
	virtual EVROverlayError SetOverlayCursor(VROverlayHandle_t ulOverlayHandle, VROverlayHandle_t ulCursorHandle) = 0;
	virtual EVROverlayError SetOverlayCursorPositionOverride(VROverlayHandle_t ulOverlayHandle, const HmdVector2_t* pvCursor) = 0;
	virtual EVROverlayError ClearOverlayCursorPositionOverride(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayTexture(VROverlayHandle_t ulOverlayHandle, const Texture_t* pTexture) = 0;
	virtual EVROverlayError ClearOverlayTexture(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetOverlayRaw(VROverlayHandle_t ulOverlayHandle, void* pvBuffer, uint32_t unWidth, uint32_t unHeight, uint32_t unBytesPerPixel) = 0;
	virtual EVROverlayError SetOverlayFromFile(VROverlayHandle_t ulOverlayHandle, const char* pchFilePath) = 0;
	virtual EVROverlayError GetOverlayTexture(VROverlayHandle_t ulOverlayHandle, void** pNativeTextureHandle, void* pNativeTextureRef, uint32_t* pWidth, uint32_t* pHeight, uint32_t* pNativeFormat, ETextureType* pAPIType, EColorSpace* pColorSpace, VRTextureBounds_t* pTextureBounds) = 0;
	virtual EVROverlayError ReleaseNativeOverlayHandle(VROverlayHandle_t ulOverlayHandle, void* pNativeTextureHandle) = 0;
	virtual EVROverlayError GetOverlayTextureSize(VROverlayHandle_t ulOverlayHandle, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual EVROverlayError CreateDashboardOverlay(const char* pchOverlayKey, const char* pchOverlayFriendlyName, VROverlayHandle_t* pMainHandle, VROverlayHandle_t* pThumbnailHandle) = 0;
	virtual bool IsDashboardVisible() = 0;
	virtual bool IsActiveDashboardOverlay(VROverlayHandle_t ulOverlayHandle) = 0;
	virtual EVROverlayError SetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t unProcessId) = 0;
	virtual EVROverlayError GetDashboardOverlaySceneProcess(VROverlayHandle_t ulOverlayHandle, uint32_t* punProcessId) = 0;
	virtual void ShowDashboard(const char* pchOverlayToShow) = 0;
	virtual vr::TrackedDeviceIndex_t GetPrimaryDashboardDevice() = 0;
	virtual EVROverlayError ShowKeyboard(EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, uint32_t unFlags, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, uint64_t uUserValue) = 0;
	virtual EVROverlayError ShowKeyboardForOverlay(VROverlayHandle_t ulOverlayHandle, EGamepadTextInputMode eInputMode, EGamepadTextInputLineMode eLineInputMode, uint32_t unFlags, const char* pchDescription, uint32_t unCharMax, const char* pchExistingText, uint64_t uUserValue) = 0;
	virtual uint32_t GetKeyboardText(VR_OUT_STRING() char* pchText, uint32_t cchText) = 0;
	virtual void HideKeyboard() = 0;
	virtual void SetKeyboardTransformAbsolute(ETrackingUniverseOrigin eTrackingOrigin, const HmdMatrix34_t* pmatTrackingOriginToKeyboardTransform) = 0;
	virtual void SetKeyboardPositionForOverlay(VROverlayHandle_t ulOverlayHandle, HmdRect2_t avoidRect) = 0;
	virtual VRMessageOverlayResponse ShowMessageOverlay(const char* pchText, const char* pchCaption, const char* pchButton0Text, const char* pchButton1Text = nullptr, const char* pchButton2Text = nullptr, const char* pchButton3Text = nullptr) = 0;
	virtual void CloseMessageOverlay() = 0;
};


static const char* const IVROverlayView_003_Version = "IVROverlayView_003";

class IVROverlayView_003
{
public:
	virtual EVROverlayError AcquireOverlayView(VROverlayHandle_t ulOverlayHandle, VRNativeDevice_t* pNativeDevice, VROverlayView_t* pOverlayView, uint32_t unOverlayViewSize) = 0;
	virtual EVROverlayError ReleaseOverlayView(VROverlayView_t* pOverlayView) = 0;
	virtual void PostOverlayEvent(VROverlayHandle_t ulOverlayHandle, const VREvent_t* pvrEvent) = 0;
	virtual bool IsViewingPermitted(VROverlayHandle_t ulOverlayHandle) = 0;
};


static const char* const IVRRenderModels_001_Version = "IVRRenderModels_001";

class IVRRenderModels_001
{
public:
	virtual bool LoadRenderModel(const char* pchRenderModelName, RenderModel_t* pRenderModel) = 0;
	virtual void FreeRenderModel(RenderModel_t* pRenderModel) = 0;
	virtual uint32_t GetRenderModelName(uint32_t unRenderModelIndex, VR_OUT_STRING() char* pchRenderModelName, uint32_t unRenderModelNameLen) = 0;
	virtual uint32_t GetRenderModelCount() = 0;
	virtual uint32_t GetComponentCount(const char* pchRenderModelName) = 0;
	virtual uint32_t GetComponentName(const char* pchRenderModelName, uint32_t unComponentIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameLen) = 0;
	virtual uint64_t GetComponentButtonMask(const char* pchRenderModelName, const char* pchComponentName) = 0;
	virtual uint32_t GetComponentRenderModelName(const char* pchRenderModelName, const char* pchComponentName, VR_OUT_STRING() char* pchComponentRenderModelName, uint32_t unComponentRenderModelNameLen) = 0;
	virtual bool GetComponentState(const char* pchRenderModelName, const char* pchComponentName, vr::VRControllerState_t controllerState, ComponentState_t* pComponentState) = 0;
};


static const char* const IVRRenderModels_002_Version = "IVRRenderModels_002";

class IVRRenderModels_002
{
public:
	virtual bool LoadRenderModel(const char* pchRenderModelName, RenderModel_t** ppRenderModel) = 0;
	virtual void FreeRenderModel(RenderModel_t* pRenderModel) = 0;
	virtual bool LoadTexture(TextureID_t textureId, RenderModel_TextureMap_t** ppTexture) = 0;
	virtual void FreeTexture(RenderModel_TextureMap_t* pTexture) = 0;
	virtual uint32_t GetRenderModelName(uint32_t unRenderModelIndex, VR_OUT_STRING() char* pchRenderModelName, uint32_t unRenderModelNameLen) = 0;
	virtual uint32_t GetRenderModelCount() = 0;
	virtual uint32_t GetComponentCount(const char* pchRenderModelName) = 0;
	virtual uint32_t GetComponentName(const char* pchRenderModelName, uint32_t unComponentIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameLen) = 0;
	virtual uint64_t GetComponentButtonMask(const char* pchRenderModelName, const char* pchComponentName) = 0;
	virtual uint32_t GetComponentRenderModelName(const char* pchRenderModelName, const char* pchComponentName, VR_OUT_STRING() char* pchComponentRenderModelName, uint32_t unComponentRenderModelNameLen) = 0;
	virtual bool GetComponentState(const char* pchRenderModelName, const char* pchComponentName, const vr::VRControllerState_t* pControllerState, RenderModel_ComponentState_t* pComponentState) = 0;
	virtual bool RenderModelHasComponent(const char* pchRenderModelName, const char* pchComponentName) = 0;
};


static const char* const IVRRenderModels_004_Version = "IVRRenderModels_004";

class IVRRenderModels_004
{
public:
	virtual EVRRenderModelError LoadRenderModel_Async(const char* pchRenderModelName, RenderModel_t** ppRenderModel) = 0;
	virtual void FreeRenderModel(RenderModel_t* pRenderModel) = 0;
	virtual EVRRenderModelError LoadTexture_Async(TextureID_t textureId, RenderModel_TextureMap_t** ppTexture) = 0;
	virtual void FreeTexture(RenderModel_TextureMap_t* pTexture) = 0;
	virtual EVRRenderModelError LoadTextureD3D11_Async(TextureID_t textureId, void* pD3D11Device, void** ppD3D11Texture2D) = 0;
	virtual void FreeTextureD3D11(void* pD3D11Texture2D) = 0;
	virtual uint32_t GetRenderModelName(uint32_t unRenderModelIndex, VR_OUT_STRING() char* pchRenderModelName, uint32_t unRenderModelNameLen) = 0;
	virtual uint32_t GetRenderModelCount() = 0;
	virtual uint32_t GetComponentCount(const char* pchRenderModelName) = 0;
	virtual uint32_t GetComponentName(const char* pchRenderModelName, uint32_t unComponentIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameLen) = 0;
	virtual uint64_t GetComponentButtonMask(const char* pchRenderModelName, const char* pchComponentName) = 0;
	virtual uint32_t GetComponentRenderModelName(const char* pchRenderModelName, const char* pchComponentName, VR_OUT_STRING() char* pchComponentRenderModelName, uint32_t unComponentRenderModelNameLen) = 0;
	virtual bool GetComponentState(const char* pchRenderModelName, const char* pchComponentName, const vr::VRControllerState_t* pControllerState, const RenderModel_ControllerMode_State_t* pState, RenderModel_ComponentState_t* pComponentState) = 0;
	virtual bool RenderModelHasComponent(const char* pchRenderModelName, const char* pchComponentName) = 0;
};


static const char* const IVRRenderModels_005_Version = "IVRRenderModels_005";

class IVRRenderModels_005
{
public:
	virtual EVRRenderModelError LoadRenderModel_Async(const char* pchRenderModelName, RenderModel_t** ppRenderModel) = 0;
	virtual void FreeRenderModel(RenderModel_t* pRenderModel) = 0;
	virtual EVRRenderModelError LoadTexture_Async(TextureID_t textureId, RenderModel_TextureMap_t** ppTexture) = 0;
	virtual void FreeTexture(RenderModel_TextureMap_t* pTexture) = 0;
	virtual EVRRenderModelError LoadTextureD3D11_Async(TextureID_t textureId, void* pD3D11Device, void** ppD3D11Texture2D) = 0;
	virtual EVRRenderModelError LoadIntoTextureD3D11_Async(TextureID_t textureId, void* pDstTexture) = 0;
	virtual void FreeTextureD3D11(void* pD3D11Texture2D) = 0;
	virtual uint32_t GetRenderModelName(uint32_t unRenderModelIndex, VR_OUT_STRING() char* pchRenderModelName, uint32_t unRenderModelNameLen) = 0;
	virtual uint32_t GetRenderModelCount() = 0;
	virtual uint32_t GetComponentCount(const char* pchRenderModelName) = 0;
	virtual uint32_t GetComponentName(const char* pchRenderModelName, uint32_t unComponentIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameLen) = 0;
	virtual uint64_t GetComponentButtonMask(const char* pchRenderModelName, const char* pchComponentName) = 0;
	virtual uint32_t GetComponentRenderModelName(const char* pchRenderModelName, const char* pchComponentName, VR_OUT_STRING() char* pchComponentRenderModelName, uint32_t unComponentRenderModelNameLen) = 0;
	virtual bool GetComponentState(const char* pchRenderModelName, const char* pchComponentName, const vr::VRControllerState_t* pControllerState, const RenderModel_ControllerMode_State_t* pState, RenderModel_ComponentState_t* pComponentState) = 0;
	virtual bool RenderModelHasComponent(const char* pchRenderModelName, const char* pchComponentName) = 0;
	virtual uint32_t GetRenderModelThumbnailURL(const char* pchRenderModelName, VR_OUT_STRING() char* pchThumbnailURL, uint32_t unThumbnailURLLen, vr::EVRRenderModelError* peError) = 0;
	virtual uint32_t GetRenderModelOriginalPath(const char* pchRenderModelName, VR_OUT_STRING() char* pchOriginalPath, uint32_t unOriginalPathLen, vr::EVRRenderModelError* peError) = 0;
	virtual const char* GetRenderModelErrorNameFromEnum(vr::EVRRenderModelError error) = 0;
};


static const char* const IVRRenderModels_006_Version = "IVRRenderModels_006";

class IVRRenderModels_006
{
public:
	virtual EVRRenderModelError LoadRenderModel_Async(const char* pchRenderModelName, RenderModel_t** ppRenderModel) = 0;
	virtual void FreeRenderModel(RenderModel_t* pRenderModel) = 0;
	virtual EVRRenderModelError LoadTexture_Async(TextureID_t textureId, RenderModel_TextureMap_t** ppTexture) = 0;
	virtual void FreeTexture(RenderModel_TextureMap_t* pTexture) = 0;
	virtual EVRRenderModelError LoadTextureD3D11_Async(TextureID_t textureId, void* pD3D11Device, void** ppD3D11Texture2D) = 0;
	virtual EVRRenderModelError LoadIntoTextureD3D11_Async(TextureID_t textureId, void* pDstTexture) = 0;
	virtual void FreeTextureD3D11(void* pD3D11Texture2D) = 0;
	virtual uint32_t GetRenderModelName(uint32_t unRenderModelIndex, VR_OUT_STRING() char* pchRenderModelName, uint32_t unRenderModelNameLen) = 0;
	virtual uint32_t GetRenderModelCount() = 0;
	virtual uint32_t GetComponentCount(const char* pchRenderModelName) = 0;
	virtual uint32_t GetComponentName(const char* pchRenderModelName, uint32_t unComponentIndex, VR_OUT_STRING() char* pchComponentName, uint32_t unComponentNameLen) = 0;
	virtual uint64_t GetComponentButtonMask(const char* pchRenderModelName, const char* pchComponentName) = 0;
	virtual uint32_t GetComponentRenderModelName(const char* pchRenderModelName, const char* pchComponentName, VR_OUT_STRING() char* pchComponentRenderModelName, uint32_t unComponentRenderModelNameLen) = 0;
	virtual bool GetComponentStateForDevicePath(const char* pchRenderModelName, const char* pchComponentName, vr::VRInputValueHandle_t devicePath, const vr::RenderModel_ControllerMode_State_t* pState, vr::RenderModel_ComponentState_t* pComponentState) = 0;
	virtual bool GetComponentState(const char* pchRenderModelName, const char* pchComponentName, const vr::VRControllerState_t* pControllerState, const RenderModel_ControllerMode_State_t* pState, RenderModel_ComponentState_t* pComponentState) = 0;
	virtual bool RenderModelHasComponent(const char* pchRenderModelName, const char* pchComponentName) = 0;
	virtual uint32_t GetRenderModelThumbnailURL(const char* pchRenderModelName, VR_OUT_STRING() char* pchThumbnailURL, uint32_t unThumbnailURLLen, vr::EVRRenderModelError* peError) = 0;
	virtual uint32_t GetRenderModelOriginalPath(const char* pchRenderModelName, VR_OUT_STRING() char* pchOriginalPath, uint32_t unOriginalPathLen, vr::EVRRenderModelError* peError) = 0;
	virtual const char* GetRenderModelErrorNameFromEnum(vr::EVRRenderModelError error) = 0;
};


static const char* const IVRResources_001_Version = "IVRResources_001";

class IVRResources_001
{
public:
	virtual uint32_t LoadSharedResource(const char* pchResourceName, char* pchBuffer, uint32_t unBufferLen) = 0;
	virtual uint32_t GetResourceFullPath(const char* pchResourceName, const char* pchResourceTypeDirectory, VR_OUT_STRING() char* pchPathBuffer, uint32_t unBufferLen) = 0;
};


static const char* const IVRScreenshots_001_Version = "IVRScreenshots_001";

class IVRScreenshots_001
{
public:
	virtual vr::EVRScreenshotError RequestScreenshot(vr::ScreenshotHandle_t* pOutScreenshotHandle, vr::EVRScreenshotType type, const char* pchPreviewFilename, const char* pchVRFilename) = 0;
	virtual vr::EVRScreenshotError HookScreenshot(VR_ARRAY_COUNT(numTypes) const vr::EVRScreenshotType* pSupportedTypes, int numTypes) = 0;
	virtual vr::EVRScreenshotType GetScreenshotPropertyType(vr::ScreenshotHandle_t screenshotHandle, vr::EVRScreenshotError* pError) = 0;
	virtual uint32_t GetScreenshotPropertyFilename(vr::ScreenshotHandle_t screenshotHandle, vr::EVRScreenshotPropertyFilenames filenameType, VR_OUT_STRING() char* pchFilename, uint32_t cchFilename, vr::EVRScreenshotError* pError) = 0;
	virtual vr::EVRScreenshotError UpdateScreenshotProgress(vr::ScreenshotHandle_t screenshotHandle, float flProgress) = 0;
	virtual vr::EVRScreenshotError TakeStereoScreenshot(vr::ScreenshotHandle_t* pOutScreenshotHandle, const char* pchPreviewFilename, const char* pchVRFilename) = 0;
	virtual vr::EVRScreenshotError SubmitScreenshot(vr::ScreenshotHandle_t screenshotHandle, vr::EVRScreenshotType type, const char* pchSourcePreviewFilename, const char* pchSourceVRFilename) = 0;
};


static const char* const IVRSettings_001_Version = "IVRSettings_001";

class IVRSettings_001
{
public:
	virtual const char* GetSettingsErrorNameFromEnum(EVRSettingsError eError) = 0;
	virtual bool Sync(bool bForce = false, EVRSettingsError* peError = nullptr) = 0;
	virtual bool GetBool(const char* pchSection, const char* pchSettingsKey, bool bDefaultValue, EVRSettingsError* peError = nullptr) = 0;
	virtual void SetBool(const char* pchSection, const char* pchSettingsKey, bool bValue, EVRSettingsError* peError = nullptr) = 0;
	virtual int32_t GetInt32(const char* pchSection, const char* pchSettingsKey, int32_t nDefaultValue, EVRSettingsError* peError = nullptr) = 0;
	virtual void SetInt32(const char* pchSection, const char* pchSettingsKey, int32_t nValue, EVRSettingsError* peError = nullptr) = 0;
	virtual float GetFloat(const char* pchSection, const char* pchSettingsKey, float flDefaultValue, EVRSettingsError* peError = nullptr) = 0;
	virtual void SetFloat(const char* pchSection, const char* pchSettingsKey, float flValue, EVRSettingsError* peError = nullptr) = 0;
	virtual void GetString(const char* pchSection, const char* pchSettingsKey, VR_OUT_STRING() char* pchValue, uint32_t unValueLen, const char* pchDefaultValue, EVRSettingsError* peError = nullptr) = 0;
	virtual void SetString(const char* pchSection, const char* pchSettingsKey, const char* pchValue, EVRSettingsError* peError = nullptr) = 0;
	virtual void RemoveSection(const char* pchSection, EVRSettingsError* peError = nullptr) = 0;
	virtual void RemoveKeyInSection(const char* pchSection, const char* pchSettingsKey, EVRSettingsError* peError = nullptr) = 0;
};


static const char* const IVRSettings_002_Version = "IVRSettings_002";

class IVRSettings_002
{
public:
	virtual const char* GetSettingsErrorNameFromEnum(EVRSettingsError eError) = 0;
	virtual bool Sync(bool bForce = false, EVRSettingsError* peError = nullptr) = 0;
	virtual void SetBool(const char* pchSection, const char* pchSettingsKey, bool bValue, EVRSettingsError* peError = nullptr) = 0;
	virtual void SetInt32(const char* pchSection, const char* pchSettingsKey, int32_t nValue, EVRSettingsError* peError = nullptr) = 0;
	virtual void SetFloat(const char* pchSection, const char* pchSettingsKey, float flValue, EVRSettingsError* peError = nullptr) = 0;
	virtual void SetString(const char* pchSection, const char* pchSettingsKey, const char* pchValue, EVRSettingsError* peError = nullptr) = 0;
	virtual bool GetBool(const char* pchSection, const char* pchSettingsKey, EVRSettingsError* peError = nullptr) = 0;
	virtual int32_t GetInt32(const char* pchSection, const char* pchSettingsKey, EVRSettingsError* peError = nullptr) = 0;
	virtual float GetFloat(const char* pchSection, const char* pchSettingsKey, EVRSettingsError* peError = nullptr) = 0;
	virtual void GetString(const char* pchSection, const char* pchSettingsKey, VR_OUT_STRING() char* pchValue, uint32_t unValueLen, EVRSettingsError* peError = nullptr) = 0;
	virtual void RemoveSection(const char* pchSection, EVRSettingsError* peError = nullptr) = 0;
	virtual void RemoveKeyInSection(const char* pchSection, const char* pchSettingsKey, EVRSettingsError* peError = nullptr) = 0;
};


static const char* const IVRSettings_003_Version = "IVRSettings_003";

class IVRSettings_003
{
public:
	virtual const char* GetSettingsErrorNameFromEnum(EVRSettingsError eError) = 0;
	virtual void SetBool(const char* pchSection, const char* pchSettingsKey, bool bValue, EVRSettingsError* peError = nullptr) = 0;
	virtual void SetInt32(const char* pchSection, const char* pchSettingsKey, int32_t nValue, EVRSettingsError* peError = nullptr) = 0;
	virtual void SetFloat(const char* pchSection, const char* pchSettingsKey, float flValue, EVRSettingsError* peError = nullptr) = 0;
	virtual void SetString(const char* pchSection, const char* pchSettingsKey, const char* pchValue, EVRSettingsError* peError = nullptr) = 0;
	virtual bool GetBool(const char* pchSection, const char* pchSettingsKey, EVRSettingsError* peError = nullptr) = 0;
	virtual int32_t GetInt32(const char* pchSection, const char* pchSettingsKey, EVRSettingsError* peError = nullptr) = 0;
	virtual float GetFloat(const char* pchSection, const char* pchSettingsKey, EVRSettingsError* peError = nullptr) = 0;
	virtual void GetString(const char* pchSection, const char* pchSettingsKey, VR_OUT_STRING() char* pchValue, uint32_t unValueLen, EVRSettingsError* peError = nullptr) = 0;
	virtual void RemoveSection(const char* pchSection, EVRSettingsError* peError = nullptr) = 0;
	virtual void RemoveKeyInSection(const char* pchSection, const char* pchSettingsKey, EVRSettingsError* peError = nullptr) = 0;
};


static const char* const IVRSpatialAnchors_001_Version = "IVRSpatialAnchors_001";

class IVRSpatialAnchors_001
{
public:
	virtual EVRSpatialAnchorError CreateSpatialAnchorFromDescriptor(const char* pchDescriptor, SpatialAnchorHandle_t* pHandleOut) = 0;
	virtual EVRSpatialAnchorError CreateSpatialAnchorFromPose(TrackedDeviceIndex_t unDeviceIndex, ETrackingUniverseOrigin eOrigin, SpatialAnchorPose_t* pPose, SpatialAnchorHandle_t* pHandleOut) = 0;
	virtual EVRSpatialAnchorError GetSpatialAnchorPose(SpatialAnchorHandle_t unHandle, ETrackingUniverseOrigin eOrigin, SpatialAnchorPose_t* pPoseOut) = 0;
	virtual EVRSpatialAnchorError GetSpatialAnchorDescriptor(SpatialAnchorHandle_t unHandle, VR_OUT_STRING() char* pchDescriptorOut, uint32_t* punDescriptorBufferLenInOut) = 0;
};


static const char* const IVRSystem_003_Version = "IVRSystem_003";

class IVRSystem_003
{
public:
	virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual void GetEyeOutputViewport(Hmd_Eye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(Hmd_Eye eEye, float fNearZ, float fFarZ, GraphicsAPIConvention eProjType) = 0;
	virtual void GetProjectionRaw(Hmd_Eye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual DistortionCoordinates_t ComputeDistortion(Hmd_Eye eEye, float fU, float fV) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(Hmd_Eye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex, int32_t* pnAdapterOutputIndex) = 0;
	virtual bool AttachToWindow(void* hWnd) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(TrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual bool LoadRenderModel(const char* pchRenderModelName, RenderModel_t* pRenderModel) = 0;
	virtual void FreeRenderModel(RenderModel_t* pRenderModel) = 0;
	virtual TrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, TrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(TrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent) = 0;
	virtual bool PollNextEventWithPose(TrackingUniverseOrigin eOrigin, vr::VREvent_t* pEvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(Hmd_Eye eEye) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState) = 0;
	virtual bool GetControllerStateWithPose(TrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool HandleControllerOverlayInteractionAsMouse(const vr::Compositor_OverlaySettings& overlaySettings, vr::HmdVector2_t vecWindowClientPositionOnScreen, vr::HmdVector2_t vecWindowClientSize, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::EVRControllerEventOutputType eOutputType) = 0;
	virtual bool CaptureInputFocus() = 0;
	virtual void ReleaseInputFocus() = 0;
	virtual bool IsInputFocusCapturedByAnotherProcess() = 0;
};


static const char* const IVRSystem_004_Version = "IVRSystem_004";

class IVRSystem_004
{
public:
	virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual void GetEyeOutputViewport(Hmd_Eye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(Hmd_Eye eEye, float fNearZ, float fFarZ, GraphicsAPIConvention eProjType) = 0;
	virtual void GetProjectionRaw(Hmd_Eye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual DistortionCoordinates_t ComputeDistortion(Hmd_Eye eEye, float fU, float fV) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(Hmd_Eye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex, int32_t* pnAdapterOutputIndex) = 0;
	virtual bool AttachToWindow(void* hWnd) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(TrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual TrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, TrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(TrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent) = 0;
	virtual bool PollNextEventWithPose(TrackingUniverseOrigin eOrigin, vr::VREvent_t* pEvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(Hmd_Eye eEye) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState) = 0;
	virtual bool GetControllerStateWithPose(TrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool CaptureInputFocus() = 0;
	virtual void ReleaseInputFocus() = 0;
	virtual bool IsInputFocusCapturedByAnotherProcess() = 0;
	virtual uint32_t DriverDebugRequest(vr::TrackedDeviceIndex_t unDeviceIndex, const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
};


static const char* const IVRSystem_005_Version = "IVRSystem_005";

class IVRSystem_005
{
public:
	virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual void GetEyeOutputViewport(Hmd_Eye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(Hmd_Eye eEye, float fNearZ, float fFarZ, GraphicsAPIConvention eProjType) = 0;
	virtual void GetProjectionRaw(Hmd_Eye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual DistortionCoordinates_t ComputeDistortion(Hmd_Eye eEye, float fU, float fV) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(Hmd_Eye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex, int32_t* pnAdapterOutputIndex) = 0;
	virtual bool AttachToWindow(void* hWnd) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(TrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(TrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual TrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, TrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(TrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent) = 0;
	virtual bool PollNextEventWithPose(TrackingUniverseOrigin eOrigin, vr::VREvent_t* pEvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(Hmd_Eye eEye) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState) = 0;
	virtual bool GetControllerStateWithPose(TrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool CaptureInputFocus() = 0;
	virtual void ReleaseInputFocus() = 0;
	virtual bool IsInputFocusCapturedByAnotherProcess() = 0;
	virtual uint32_t DriverDebugRequest(vr::TrackedDeviceIndex_t unDeviceIndex, const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
};


static const char* const IVRSystem_006_Version = "IVRSystem_006";

class IVRSystem_006
{
public:
	virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual void GetEyeOutputViewport(Hmd_Eye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(Hmd_Eye eEye, float fNearZ, float fFarZ, GraphicsAPIConvention eProjType) = 0;
	virtual void GetProjectionRaw(Hmd_Eye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual DistortionCoordinates_t ComputeDistortion(Hmd_Eye eEye, float fU, float fV) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(Hmd_Eye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex, int32_t* pnAdapterOutputIndex) = 0;
	virtual bool AttachToWindow(void* hWnd) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(TrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(TrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual TrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, TrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(TrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent) = 0;
	virtual bool PollNextEventWithPose(TrackingUniverseOrigin eOrigin, vr::VREvent_t* pEvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(Hmd_Eye eEye) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState) = 0;
	virtual bool GetControllerStateWithPose(TrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool CaptureInputFocus() = 0;
	virtual void ReleaseInputFocus() = 0;
	virtual bool IsInputFocusCapturedByAnotherProcess() = 0;
	virtual uint32_t DriverDebugRequest(vr::TrackedDeviceIndex_t unDeviceIndex, const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
	virtual vr::VRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
};


static const char* const IVRSystem_007_Version = "IVRSystem_007";

class IVRSystem_007
{
public:
	virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual void GetEyeOutputViewport(Hmd_Eye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(Hmd_Eye eEye, float fNearZ, float fFarZ, GraphicsAPIConvention eProjType) = 0;
	virtual void GetProjectionRaw(Hmd_Eye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual DistortionCoordinates_t ComputeDistortion(Hmd_Eye eEye, float fU, float fV) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(Hmd_Eye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex, int32_t* pnAdapterOutputIndex) = 0;
	virtual bool AttachToWindow(void* hWnd) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(TrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(TrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t& trackedDevicePose, const HmdMatrix34_t& transform) = 0;
	virtual TrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, TrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, TrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(TrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent) = 0;
	virtual bool PollNextEventWithPose(TrackingUniverseOrigin eOrigin, vr::VREvent_t* pEvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(Hmd_Eye eEye) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState) = 0;
	virtual bool GetControllerStateWithPose(TrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool CaptureInputFocus() = 0;
	virtual void ReleaseInputFocus() = 0;
	virtual bool IsInputFocusCapturedByAnotherProcess() = 0;
	virtual uint32_t DriverDebugRequest(vr::TrackedDeviceIndex_t unDeviceIndex, const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
	virtual vr::VRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
};


static const char* const IVRSystem_009_Version = "IVRSystem_009";

class IVRSystem_009
{
public:
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(EVREye eEye, float fNearZ, float fFarZ, EGraphicsAPIConvention eProjType) = 0;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual DistortionCoordinates_t ComputeDistortion(EVREye eEye, float fU, float fV) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(EVREye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(ETrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t* pTrackedDevicePose, const HmdMatrix34_t* pTransform) = 0;
	virtual ETrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(ETrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent) = 0;
	virtual bool PollNextEventWithPose(ETrackingUniverseOrigin eOrigin, vr::VREvent_t* pEvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(EVREye eEye) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState) = 0;
	virtual bool GetControllerStateWithPose(ETrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool CaptureInputFocus() = 0;
	virtual void ReleaseInputFocus() = 0;
	virtual bool IsInputFocusCapturedByAnotherProcess() = 0;
	virtual uint32_t DriverDebugRequest(vr::TrackedDeviceIndex_t unDeviceIndex, const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
	virtual vr::EVRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual void AcknowledgeQuit_Exiting() = 0;
	virtual void AcknowledgeQuit_UserPrompt() = 0;
};


static const char* const IVRSystem_010_Version = "IVRSystem_010";

class IVRSystem_010
{
public:
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(EVREye eEye, float fNearZ, float fFarZ, EGraphicsAPIConvention eProjType) = 0;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual DistortionCoordinates_t ComputeDistortion(EVREye eEye, float fU, float fV) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(EVREye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(ETrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t* pTrackedDevicePose, const HmdMatrix34_t* pTransform) = 0;
	virtual vr::TrackedDeviceIndex_t GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole unDeviceType) = 0;
	virtual vr::ETrackedControllerRole GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual ETrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(ETrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent) = 0;
	virtual bool PollNextEventWithPose(ETrackingUniverseOrigin eOrigin, vr::VREvent_t* pEvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(EVREye eEye) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState) = 0;
	virtual bool GetControllerStateWithPose(ETrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool CaptureInputFocus() = 0;
	virtual void ReleaseInputFocus() = 0;
	virtual bool IsInputFocusCapturedByAnotherProcess() = 0;
	virtual uint32_t DriverDebugRequest(vr::TrackedDeviceIndex_t unDeviceIndex, const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
	virtual vr::EVRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual void AcknowledgeQuit_Exiting() = 0;
	virtual void AcknowledgeQuit_UserPrompt() = 0;
	virtual void PerformanceTestEnableCapture(bool bEnable) = 0;
	virtual void PerformanceTestReportFidelityLevelChange(int nFidelityLevel) = 0;
};


static const char* const IVRSystem_011_Version = "IVRSystem_011";

class IVRSystem_011
{
public:
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(EVREye eEye, float fNearZ, float fFarZ, EGraphicsAPIConvention eProjType) = 0;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual DistortionCoordinates_t ComputeDistortion(EVREye eEye, float fU, float fV) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(EVREye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(ETrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t* pTrackedDevicePose, const HmdMatrix34_t* pTransform) = 0;
	virtual vr::TrackedDeviceIndex_t GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole unDeviceType) = 0;
	virtual vr::ETrackedControllerRole GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual ETrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(ETrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual bool PollNextEventWithPose(ETrackingUniverseOrigin eOrigin, VREvent_t* pEvent, uint32_t uncbVREvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(EVREye eEye) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState) = 0;
	virtual bool GetControllerStateWithPose(ETrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool CaptureInputFocus() = 0;
	virtual void ReleaseInputFocus() = 0;
	virtual bool IsInputFocusCapturedByAnotherProcess() = 0;
	virtual uint32_t DriverDebugRequest(vr::TrackedDeviceIndex_t unDeviceIndex, const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
	virtual vr::EVRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual void AcknowledgeQuit_Exiting() = 0;
	virtual void AcknowledgeQuit_UserPrompt() = 0;
	virtual void PerformanceTestEnableCapture(bool bEnable) = 0;
	virtual void PerformanceTestReportFidelityLevelChange(int nFidelityLevel) = 0;
};


static const char* const IVRSystem_012_Version = "IVRSystem_012";

class IVRSystem_012
{
public:
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(EVREye eEye, float fNearZ, float fFarZ, EGraphicsAPIConvention eProjType) = 0;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual DistortionCoordinates_t ComputeDistortion(EVREye eEye, float fU, float fV) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(EVREye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(ETrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t* pTrackedDevicePose, const HmdMatrix34_t* pTransform) = 0;
	virtual vr::TrackedDeviceIndex_t GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole unDeviceType) = 0;
	virtual vr::ETrackedControllerRole GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual ETrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(ETrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual bool PollNextEventWithPose(ETrackingUniverseOrigin eOrigin, VREvent_t* pEvent, uint32_t uncbVREvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(EVREye eEye) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState) = 0;
	virtual bool GetControllerStateWithPose(ETrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool CaptureInputFocus() = 0;
	virtual void ReleaseInputFocus() = 0;
	virtual bool IsInputFocusCapturedByAnotherProcess() = 0;
	virtual uint32_t DriverDebugRequest(vr::TrackedDeviceIndex_t unDeviceIndex, const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
	virtual vr::EVRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual void AcknowledgeQuit_Exiting() = 0;
	virtual void AcknowledgeQuit_UserPrompt() = 0;
};


static const char* const IVRSystem_014_Version = "IVRSystem_014";

class IVRSystem_014
{
public:
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(EVREye eEye, float fNearZ, float fFarZ, EGraphicsAPIConvention eProjType) = 0;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual bool ComputeDistortion(EVREye eEye, float fU, float fV, DistortionCoordinates_t* pDistortionCoordinates) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(EVREye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(ETrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t* pTrackedDevicePose, const HmdMatrix34_t* pTransform) = 0;
	virtual vr::TrackedDeviceIndex_t GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole unDeviceType) = 0;
	virtual vr::ETrackedControllerRole GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual ETrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(ETrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual bool PollNextEventWithPose(ETrackingUniverseOrigin eOrigin, VREvent_t* pEvent, uint32_t uncbVREvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(EVREye eEye, EHiddenAreaMeshType type = k_eHiddenAreaMesh_Standard) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize) = 0;
	virtual bool GetControllerStateWithPose(ETrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool CaptureInputFocus() = 0;
	virtual void ReleaseInputFocus() = 0;
	virtual bool IsInputFocusCapturedByAnotherProcess() = 0;
	virtual uint32_t DriverDebugRequest(vr::TrackedDeviceIndex_t unDeviceIndex, const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
	virtual vr::EVRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual void AcknowledgeQuit_Exiting() = 0;
	virtual void AcknowledgeQuit_UserPrompt() = 0;
};


static const char* const IVRSystem_015_Version = "IVRSystem_015";

class IVRSystem_015
{
public:
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(EVREye eEye, float fNearZ, float fFarZ) = 0;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual bool ComputeDistortion(EVREye eEye, float fU, float fV, DistortionCoordinates_t* pDistortionCoordinates) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(EVREye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(ETrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t* pTrackedDevicePose, const HmdMatrix34_t* pTransform) = 0;
	virtual vr::TrackedDeviceIndex_t GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole unDeviceType) = 0;
	virtual vr::ETrackedControllerRole GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual ETrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(ETrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual bool PollNextEventWithPose(ETrackingUniverseOrigin eOrigin, VREvent_t* pEvent, uint32_t uncbVREvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(EVREye eEye, EHiddenAreaMeshType type = k_eHiddenAreaMesh_Standard) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize) = 0;
	virtual bool GetControllerStateWithPose(ETrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool CaptureInputFocus() = 0;
	virtual void ReleaseInputFocus() = 0;
	virtual bool IsInputFocusCapturedByAnotherProcess() = 0;
	virtual uint32_t DriverDebugRequest(vr::TrackedDeviceIndex_t unDeviceIndex, const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
	virtual vr::EVRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual void AcknowledgeQuit_Exiting() = 0;
	virtual void AcknowledgeQuit_UserPrompt() = 0;
};


static const char* const IVRSystem_016_Version = "IVRSystem_016";

class IVRSystem_016
{
public:
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(EVREye eEye, float fNearZ, float fFarZ) = 0;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual bool ComputeDistortion(EVREye eEye, float fU, float fV, DistortionCoordinates_t* pDistortionCoordinates) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(EVREye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex) = 0;
	virtual void GetOutputDevice(uint64_t* pnDevice, ETextureType textureType) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(ETrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t* pTrackedDevicePose, const HmdMatrix34_t* pTransform) = 0;
	virtual vr::TrackedDeviceIndex_t GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole unDeviceType) = 0;
	virtual vr::ETrackedControllerRole GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual ETrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(ETrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual bool PollNextEventWithPose(ETrackingUniverseOrigin eOrigin, VREvent_t* pEvent, uint32_t uncbVREvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(EVREye eEye, EHiddenAreaMeshType type = k_eHiddenAreaMesh_Standard) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize) = 0;
	virtual bool GetControllerStateWithPose(ETrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool CaptureInputFocus() = 0;
	virtual void ReleaseInputFocus() = 0;
	virtual bool IsInputFocusCapturedByAnotherProcess() = 0;
	virtual uint32_t DriverDebugRequest(vr::TrackedDeviceIndex_t unDeviceIndex, const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
	virtual vr::EVRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual void AcknowledgeQuit_Exiting() = 0;
	virtual void AcknowledgeQuit_UserPrompt() = 0;
};


static const char* const IVRSystem_017_Version = "IVRSystem_017";

class IVRSystem_017
{
public:
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(EVREye eEye, float fNearZ, float fFarZ) = 0;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual bool ComputeDistortion(EVREye eEye, float fU, float fV, DistortionCoordinates_t* pDistortionCoordinates) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(EVREye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex) = 0;
	virtual void GetOutputDevice(uint64_t* pnDevice, ETextureType textureType, VkInstance_T* pInstance = nullptr) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(ETrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t* pTrackedDevicePose, const HmdMatrix34_t* pTransform) = 0;
	virtual vr::TrackedDeviceIndex_t GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole unDeviceType) = 0;
	virtual vr::ETrackedControllerRole GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual ETrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(ETrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual bool PollNextEventWithPose(ETrackingUniverseOrigin eOrigin, VREvent_t* pEvent, uint32_t uncbVREvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(EVREye eEye, EHiddenAreaMeshType type = k_eHiddenAreaMesh_Standard) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize) = 0;
	virtual bool GetControllerStateWithPose(ETrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool CaptureInputFocus() = 0;
	virtual void ReleaseInputFocus() = 0;
	virtual bool IsInputFocusCapturedByAnotherProcess() = 0;
	virtual uint32_t DriverDebugRequest(vr::TrackedDeviceIndex_t unDeviceIndex, const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
	virtual vr::EVRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual void AcknowledgeQuit_Exiting() = 0;
	virtual void AcknowledgeQuit_UserPrompt() = 0;
};


static const char* const IVRSystem_019_Version = "IVRSystem_019";

class IVRSystem_019
{
public:
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(EVREye eEye, float fNearZ, float fFarZ) = 0;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual bool ComputeDistortion(EVREye eEye, float fU, float fV, DistortionCoordinates_t* pDistortionCoordinates) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(EVREye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex) = 0;
	virtual void GetOutputDevice(uint64_t* pnDevice, ETextureType textureType, VkInstance_T* pInstance = nullptr) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(ETrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t* pTrackedDevicePose, const HmdMatrix34_t* pTransform) = 0;
	virtual vr::TrackedDeviceIndex_t GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole unDeviceType) = 0;
	virtual vr::ETrackedControllerRole GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual ETrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetArrayTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, PropertyTypeTag_t propType, void* pBuffer, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(ETrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual bool PollNextEventWithPose(ETrackingUniverseOrigin eOrigin, VREvent_t* pEvent, uint32_t uncbVREvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(EVREye eEye, EHiddenAreaMeshType type = k_eHiddenAreaMesh_Standard) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize) = 0;
	virtual bool GetControllerStateWithPose(ETrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool IsInputAvailable() = 0;
	virtual bool IsSteamVRDrawingControllers() = 0;
	virtual bool ShouldApplicationPause() = 0;
	virtual bool ShouldApplicationReduceRenderingWork() = 0;
	virtual uint32_t DriverDebugRequest(vr::TrackedDeviceIndex_t unDeviceIndex, const char* pchRequest, VR_OUT_STRING() char* pchResponseBuffer, uint32_t unResponseBufferSize) = 0;
	virtual vr::EVRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual void AcknowledgeQuit_Exiting() = 0;
	virtual void AcknowledgeQuit_UserPrompt() = 0;
};


static const char* const IVRSystem_020_Version = "IVRSystem_020";

class IVRSystem_020
{
public:
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(EVREye eEye, float fNearZ, float fFarZ) = 0;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual bool ComputeDistortion(EVREye eEye, float fU, float fV, DistortionCoordinates_t* pDistortionCoordinates) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(EVREye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex) = 0;
	virtual void GetOutputDevice(uint64_t* pnDevice, ETextureType textureType, VkInstance_T* pInstance = nullptr) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(ETrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t* pTrackedDevicePose, const HmdMatrix34_t* pTransform) = 0;
	virtual vr::TrackedDeviceIndex_t GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole unDeviceType) = 0;
	virtual vr::ETrackedControllerRole GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual ETrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetArrayTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, PropertyTypeTag_t propType, void* pBuffer, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(ETrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual bool PollNextEventWithPose(ETrackingUniverseOrigin eOrigin, VREvent_t* pEvent, uint32_t uncbVREvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(EVREye eEye, EHiddenAreaMeshType type = k_eHiddenAreaMesh_Standard) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize) = 0;
	virtual bool GetControllerStateWithPose(ETrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool IsInputAvailable() = 0;
	virtual bool IsSteamVRDrawingControllers() = 0;
	virtual bool ShouldApplicationPause() = 0;
	virtual bool ShouldApplicationReduceRenderingWork() = 0;
	virtual vr::EVRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual void AcknowledgeQuit_Exiting() = 0;
	virtual void AcknowledgeQuit_UserPrompt() = 0;
	virtual uint32_t GetAppContainerFilePaths(VR_OUT_STRING() char* pchBuffer, uint32_t unBufferSize) = 0;
	virtual const char* GetRuntimeVersion() = 0;
};


static const char* const IVRSystem_021_Version = "IVRSystem_021";

class IVRSystem_021
{
public:
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(EVREye eEye, float fNearZ, float fFarZ) = 0;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual bool ComputeDistortion(EVREye eEye, float fU, float fV, DistortionCoordinates_t* pDistortionCoordinates) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(EVREye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex) = 0;
	virtual void GetOutputDevice(uint64_t* pnDevice, ETextureType textureType, VkInstance_T* pInstance = nullptr) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual void ResetSeatedZeroPose() = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(ETrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t* pTrackedDevicePose, const HmdMatrix34_t* pTransform) = 0;
	virtual vr::TrackedDeviceIndex_t GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole unDeviceType) = 0;
	virtual vr::ETrackedControllerRole GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual ETrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetArrayTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, PropertyTypeTag_t propType, void* pBuffer, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(ETrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual bool PollNextEventWithPose(ETrackingUniverseOrigin eOrigin, VREvent_t* pEvent, uint32_t uncbVREvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(EVREye eEye, EHiddenAreaMeshType type = k_eHiddenAreaMesh_Standard) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize) = 0;
	virtual bool GetControllerStateWithPose(ETrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool IsInputAvailable() = 0;
	virtual bool IsSteamVRDrawingControllers() = 0;
	virtual bool ShouldApplicationPause() = 0;
	virtual bool ShouldApplicationReduceRenderingWork() = 0;
	virtual vr::EVRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual void AcknowledgeQuit_Exiting() = 0;
	virtual uint32_t GetAppContainerFilePaths(VR_OUT_STRING() char* pchBuffer, uint32_t unBufferSize) = 0;
	virtual const char* GetRuntimeVersion() = 0;
};


static const char* const IVRSystem_022_Version = "IVRSystem_022";

class IVRSystem_022
{
public:
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(EVREye eEye, float fNearZ, float fFarZ) = 0;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual bool ComputeDistortion(EVREye eEye, float fU, float fV, DistortionCoordinates_t* pDistortionCoordinates) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(EVREye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex) = 0;
	virtual void GetOutputDevice(uint64_t* pnDevice, ETextureType textureType, VkInstance_T* pInstance = nullptr) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(ETrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t* pTrackedDevicePose, const HmdMatrix34_t* pTransform) = 0;
	virtual vr::TrackedDeviceIndex_t GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole unDeviceType) = 0;
	virtual vr::ETrackedControllerRole GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual ETrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetArrayTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, PropertyTypeTag_t propType, void* pBuffer, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(ETrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual bool PollNextEventWithPose(ETrackingUniverseOrigin eOrigin, VREvent_t* pEvent, uint32_t uncbVREvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(EVREye eEye, EHiddenAreaMeshType type = k_eHiddenAreaMesh_Standard) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize) = 0;
	virtual bool GetControllerStateWithPose(ETrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool IsInputAvailable() = 0;
	virtual bool IsSteamVRDrawingControllers() = 0;
	virtual bool ShouldApplicationPause() = 0;
	virtual bool ShouldApplicationReduceRenderingWork() = 0;
	virtual vr::EVRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual void AcknowledgeQuit_Exiting() = 0;
	virtual uint32_t GetAppContainerFilePaths(VR_OUT_STRING() char* pchBuffer, uint32_t unBufferSize) = 0;
	virtual const char* GetRuntimeVersion() = 0;
};


static const char* const IVRSystem_023_Version = "IVRSystem_023";

class IVRSystem_023
{
public:
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(EVREye eEye, float fNearZ, float fFarZ) = 0;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual bool ComputeDistortion(EVREye eEye, float fU, float fV, DistortionCoordinates_t* pDistortionCoordinates) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(EVREye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex) = 0;
	virtual void GetOutputDevice(uint64_t* pnDevice, ETextureType textureType, VkInstance_T* pInstance = nullptr) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(ETrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t* pTrackedDevicePose, const HmdMatrix34_t* pTransform) = 0;
	virtual vr::TrackedDeviceIndex_t GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole unDeviceType) = 0;
	virtual vr::ETrackedControllerRole GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual ETrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetArrayTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, PropertyTypeTag_t propType, void* pBuffer, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(ETrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual bool PollNextEventWithPose(ETrackingUniverseOrigin eOrigin, VREvent_t* pEvent, uint32_t uncbVREvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual bool PollNextEventWithPoseAndOverlays(vr::ETrackingUniverseOrigin eOrigin, VREvent_t* pEvent, uint32_t uncbVREvent, TrackedDevicePose_t* pTrackedDevicePose, VROverlayHandle_t* pulOverlayHandle) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(EVREye eEye, EHiddenAreaMeshType type = k_eHiddenAreaMesh_Standard) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize) = 0;
	virtual bool GetControllerStateWithPose(ETrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool IsInputAvailable() = 0;
	virtual bool IsSteamVRDrawingControllers() = 0;
	virtual bool ShouldApplicationPause() = 0;
	virtual bool ShouldApplicationReduceRenderingWork() = 0;
	virtual vr::EVRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual void AcknowledgeQuit_Exiting() = 0;
	virtual uint32_t GetAppContainerFilePaths(VR_OUT_STRING() char* pchBuffer, uint32_t unBufferSize) = 0;
	virtual const char* GetRuntimeVersion() = 0;
};


static const char* const IVRSystem_026_Version = "IVRSystem_026";

class IVRSystem_026
{
public:
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual HmdMatrix44_t GetProjectionMatrix(EVREye eEye, float fNearZ, float fFarZ) = 0;
	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) = 0;
	virtual bool ComputeDistortion(EVREye eEye, float fU, float fV, DistortionCoordinates_t* pDistortionCoordinates) = 0;
	virtual bool ComputeDistortionSet(EVREye eEye, EVRDistortionChannel eChannel, bool bAsNormalizedDeviceCoordinates, uint32_t nNumCoordinates, const DistortionCoordinate_t* pInput, DistortionCoordinate_t* pOutput) = 0;
	virtual HmdMatrix34_t GetEyeToHeadTransform(EVREye eEye) = 0;
	virtual bool GetTimeSinceLastVsync(float* pfSecondsSinceLastVsync, uint64_t* pulFrameCounter) = 0;
	virtual int32_t GetD3D9AdapterIndex() = 0;
	virtual void GetDXGIOutputInfo(int32_t* pnAdapterIndex) = 0;
	virtual void GetOutputDevice(uint64_t* pnDevice, ETextureType textureType, VkInstance_T* pInstance = nullptr) = 0;
	virtual bool IsDisplayOnDesktop() = 0;
	virtual bool SetDisplayVisibility(bool bIsVisibleOnDesktop) = 0;
	virtual void GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin eOrigin, float fPredictedSecondsToPhotonsFromNow, VR_ARRAY_COUNT(unTrackedDevicePoseArrayCount) TrackedDevicePose_t* pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) = 0;
	virtual HmdMatrix34_t GetSeatedZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual HmdMatrix34_t GetRawZeroPoseToStandingAbsoluteTrackingPose() = 0;
	virtual uint32_t GetSortedTrackedDeviceIndicesOfClass(ETrackedDeviceClass eTrackedDeviceClass, VR_ARRAY_COUNT(unTrackedDeviceIndexArrayCount) vr::TrackedDeviceIndex_t* punTrackedDeviceIndexArray, uint32_t unTrackedDeviceIndexArrayCount, vr::TrackedDeviceIndex_t unRelativeToTrackedDeviceIndex = k_unTrackedDeviceIndex_Hmd) = 0;
	virtual EDeviceActivityLevel GetTrackedDeviceActivityLevel(vr::TrackedDeviceIndex_t unDeviceId) = 0;
	virtual void ApplyTransform(TrackedDevicePose_t* pOutputPose, const TrackedDevicePose_t* pTrackedDevicePose, const HmdMatrix34_t* pTransform) = 0;
	virtual vr::TrackedDeviceIndex_t GetTrackedDeviceIndexForControllerRole(vr::ETrackedControllerRole unDeviceType) = 0;
	virtual vr::ETrackedControllerRole GetControllerRoleForTrackedDeviceIndex(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual ETrackedDeviceClass GetTrackedDeviceClass(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool IsTrackedDeviceConnected(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual bool GetBoolTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual float GetFloatTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual int32_t GetInt32TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint64_t GetUint64TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetArrayTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, PropertyTypeTag_t propType, void* pBuffer, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, ETrackedDeviceProperty prop, VR_OUT_STRING() char* pchValue, uint32_t unBufferSize, ETrackedPropertyError* pError = 0L) = 0;
	virtual const char* GetPropErrorNameFromEnum(ETrackedPropertyError error) = 0;
	virtual bool PollNextEvent(VREvent_t* pEvent, uint32_t uncbVREvent) = 0;
	virtual bool PollNextEventWithPose(ETrackingUniverseOrigin eOrigin, VREvent_t* pEvent, uint32_t uncbVREvent, vr::TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual bool PollNextEventWithPoseAndOverlays(vr::ETrackingUniverseOrigin eOrigin, VREvent_t* pEvent, uint32_t uncbVREvent, TrackedDevicePose_t* pTrackedDevicePose, VROverlayHandle_t* pulOverlayHandle) = 0;
	virtual const char* GetEventTypeNameFromEnum(EVREventType eType) = 0;
	virtual HiddenAreaMesh_t GetHiddenAreaMesh(EVREye eEye, EHiddenAreaMeshType type = k_eHiddenAreaMesh_Standard) = 0;
	virtual bool GetEyeTrackedFoveationCenter(HmdVector2_t* pNdcLeft, HmdVector2_t* pNdcRight) = 0;
	virtual bool GetEyeTrackedFoveationCenterForProjection(const HmdMatrix44_t* pProjMat, HmdVector2_t* pNdc) = 0;
	virtual bool GetControllerState(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize) = 0;
	virtual bool GetControllerStateWithPose(ETrackingUniverseOrigin eOrigin, vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t* pControllerState, uint32_t unControllerStateSize, TrackedDevicePose_t* pTrackedDevicePose) = 0;
	virtual void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId, unsigned short usDurationMicroSec) = 0;
	virtual const char* GetButtonIdNameFromEnum(EVRButtonId eButtonId) = 0;
	virtual const char* GetControllerAxisTypeNameFromEnum(EVRControllerAxisType eAxisType) = 0;
	virtual bool IsInputAvailable() = 0;
	virtual bool IsSteamVRDrawingControllers() = 0;
	virtual bool ShouldApplicationPause() = 0;
	virtual bool ShouldApplicationReduceRenderingWork() = 0;
	virtual vr::EVRFirmwareError PerformFirmwareUpdate(vr::TrackedDeviceIndex_t unDeviceIndex) = 0;
	virtual void AcknowledgeQuit_Exiting() = 0;
	virtual uint32_t GetAppContainerFilePaths(VR_OUT_STRING() char* pchBuffer, uint32_t unBufferSize) = 0;
	virtual const char* GetRuntimeVersion() = 0;
	virtual vr::EVRInitError SetSDKVersion(uint32_t nVersionMajor, uint32_t nVersionMinor, uint32_t nVersionBuild) = 0;
};


static const char* const IVRTrackedCamera_001_Version = "IVRTrackedCamera_001";

class IVRTrackedCamera_001
{
public:
	virtual bool HasCamera(vr::TrackedDeviceIndex_t nDeviceIndex) = 0;
	virtual bool GetCameraFirmwareDescription(vr::TrackedDeviceIndex_t nDeviceIndex, char* pBuffer, uint32_t nBufferLen) = 0;
	virtual bool GetCameraFrameDimensions(vr::TrackedDeviceIndex_t nDeviceIndex, vr::ECameraVideoStreamFormat nVideoStreamFormat, uint32_t* pWidth, uint32_t* pHeight) = 0;
	virtual bool SetCameraVideoStreamFormat(vr::TrackedDeviceIndex_t nDeviceIndex, vr::ECameraVideoStreamFormat nVideoStreamFormat) = 0;
	virtual vr::ECameraVideoStreamFormat GetCameraVideoStreamFormat(vr::TrackedDeviceIndex_t nDeviceIndex) = 0;
	virtual bool EnableCameraForStreaming(vr::TrackedDeviceIndex_t nDeviceIndex, bool bEnable) = 0;
	virtual bool StartVideoStream(vr::TrackedDeviceIndex_t nDeviceIndex) = 0;
	virtual bool StopVideoStream(vr::TrackedDeviceIndex_t nDeviceIndex) = 0;
	virtual bool IsVideoStreamActive(vr::TrackedDeviceIndex_t nDeviceIndex) = 0;
	virtual float GetVideoStreamElapsedTime(vr::TrackedDeviceIndex_t nDeviceIndex) = 0;
	virtual const vr::CameraVideoStreamFrame_t* GetVideoStreamFrame(vr::TrackedDeviceIndex_t nDeviceIndex) = 0;
	virtual bool ReleaseVideoStreamFrame(vr::TrackedDeviceIndex_t nDeviceIndex, const vr::CameraVideoStreamFrame_t* pFrameImage) = 0;
	virtual bool SetAutoExposure(vr::TrackedDeviceIndex_t nDeviceIndex, bool bEnable) = 0;
	virtual bool PauseVideoStream(vr::TrackedDeviceIndex_t nDeviceIndex) = 0;
	virtual bool ResumeVideoStream(vr::TrackedDeviceIndex_t nDeviceIndex) = 0;
	virtual bool IsVideoStreamPaused(vr::TrackedDeviceIndex_t nDeviceIndex) = 0;
	virtual bool GetCameraDistortion(vr::TrackedDeviceIndex_t nDeviceIndex, float flInputU, float flInputV, float* pflOutputU, float* pflOutputV) = 0;
	virtual bool GetCameraProjection(vr::TrackedDeviceIndex_t nDeviceIndex, float flWidthPixels, float flHeightPixels, float flZNear, float flZFar, vr::HmdMatrix44_t* pProjection) = 0;
};


static const char* const IVRTrackedCamera_002_Version = "IVRTrackedCamera_002";

class IVRTrackedCamera_002
{
public:
	virtual const char* GetCameraErrorNameFromEnum(vr::EVRTrackedCameraError eCameraError) = 0;
	virtual vr::EVRTrackedCameraError HasCamera(vr::TrackedDeviceIndex_t nDeviceIndex, bool* pHasCamera) = 0;
	virtual vr::EVRTrackedCameraError GetCameraFrameSize(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, uint32_t* pnWidth, uint32_t* pnHeight, uint32_t* pnFrameBufferSize) = 0;
	virtual vr::EVRTrackedCameraError GetCameraIntrinisics(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, vr::HmdVector2_t* pFocalLength, vr::HmdVector2_t* pCenter) = 0;
	virtual vr::EVRTrackedCameraError GetCameraProjection(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, float flZNear, float flZFar, vr::HmdMatrix44_t* pProjection) = 0;
	virtual vr::EVRTrackedCameraError AcquireVideoStreamingService(vr::TrackedDeviceIndex_t nDeviceIndex, vr::TrackedCameraHandle_t* pHandle) = 0;
	virtual vr::EVRTrackedCameraError ReleaseVideoStreamingService(vr::TrackedCameraHandle_t hTrackedCamera) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamFrameBuffer(vr::TrackedCameraHandle_t hTrackedCamera, vr::EVRTrackedCameraFrameType eFrameType, void* pFrameBuffer, uint32_t nFrameBufferSize, vr::CameraVideoStreamFrameHeader_t* pFrameHeader, uint32_t nFrameHeaderSize) = 0;
};


static const char* const IVRTrackedCamera_003_Version = "IVRTrackedCamera_003";

class IVRTrackedCamera_003
{
public:
	virtual const char* GetCameraErrorNameFromEnum(vr::EVRTrackedCameraError eCameraError) = 0;
	virtual vr::EVRTrackedCameraError HasCamera(vr::TrackedDeviceIndex_t nDeviceIndex, bool* pHasCamera) = 0;
	virtual vr::EVRTrackedCameraError GetCameraFrameSize(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, uint32_t* pnWidth, uint32_t* pnHeight, uint32_t* pnFrameBufferSize) = 0;
	virtual vr::EVRTrackedCameraError GetCameraIntrinsics(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, vr::HmdVector2_t* pFocalLength, vr::HmdVector2_t* pCenter) = 0;
	virtual vr::EVRTrackedCameraError GetCameraProjection(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, float flZNear, float flZFar, vr::HmdMatrix44_t* pProjection) = 0;
	virtual vr::EVRTrackedCameraError AcquireVideoStreamingService(vr::TrackedDeviceIndex_t nDeviceIndex, vr::TrackedCameraHandle_t* pHandle) = 0;
	virtual vr::EVRTrackedCameraError ReleaseVideoStreamingService(vr::TrackedCameraHandle_t hTrackedCamera) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamFrameBuffer(vr::TrackedCameraHandle_t hTrackedCamera, vr::EVRTrackedCameraFrameType eFrameType, void* pFrameBuffer, uint32_t nFrameBufferSize, vr::CameraVideoStreamFrameHeader_t* pFrameHeader, uint32_t nFrameHeaderSize) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamTextureSize(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, vr::VRTextureBounds_t* pTextureBounds, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamTextureD3D11(vr::TrackedCameraHandle_t hTrackedCamera, vr::EVRTrackedCameraFrameType eFrameType, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView, vr::CameraVideoStreamFrameHeader_t* pFrameHeader, uint32_t nFrameHeaderSize) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamTextureGL(vr::TrackedCameraHandle_t hTrackedCamera, vr::EVRTrackedCameraFrameType eFrameType, vr::glUInt_t* pglTextureId, vr::CameraVideoStreamFrameHeader_t* pFrameHeader, uint32_t nFrameHeaderSize) = 0;
	virtual vr::EVRTrackedCameraError ReleaseVideoStreamTextureGL(vr::TrackedCameraHandle_t hTrackedCamera, vr::glUInt_t glTextureId) = 0;
};


static const char* const IVRTrackedCamera_004_Version = "IVRTrackedCamera_004";

class IVRTrackedCamera_004
{
public:
	virtual const char* GetCameraErrorNameFromEnum(vr::EVRTrackedCameraError eCameraError) = 0;
	virtual vr::EVRTrackedCameraError HasCamera(vr::TrackedDeviceIndex_t nDeviceIndex, bool* pHasCamera) = 0;
	virtual vr::EVRTrackedCameraError GetCameraFrameSize(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, uint32_t* pnWidth, uint32_t* pnHeight, uint32_t* pnFrameBufferSize) = 0;
	virtual vr::EVRTrackedCameraError GetCameraIntrinsics(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, vr::HmdVector2_t* pFocalLength, vr::HmdVector2_t* pCenter) = 0;
	virtual vr::EVRTrackedCameraError GetCameraProjection(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, float flZNear, float flZFar, vr::HmdMatrix44_t* pProjection) = 0;
	virtual vr::EVRTrackedCameraError AcquireVideoStreamingService(vr::TrackedDeviceIndex_t nDeviceIndex, vr::TrackedCameraHandle_t* pHandle) = 0;
	virtual vr::EVRTrackedCameraError ReleaseVideoStreamingService(vr::TrackedCameraHandle_t hTrackedCamera) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamFrameBuffer(vr::TrackedCameraHandle_t hTrackedCamera, vr::EVRTrackedCameraFrameType eFrameType, void* pFrameBuffer, uint32_t nFrameBufferSize, vr::CameraVideoStreamFrameHeader_t* pFrameHeader, uint32_t nFrameHeaderSize) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamTextureSize(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, vr::VRTextureBounds_t* pTextureBounds, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamTextureD3D11(vr::TrackedCameraHandle_t hTrackedCamera, vr::EVRTrackedCameraFrameType eFrameType, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView, vr::CameraVideoStreamFrameHeader_t* pFrameHeader, uint32_t nFrameHeaderSize) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamTextureGL(vr::TrackedCameraHandle_t hTrackedCamera, vr::EVRTrackedCameraFrameType eFrameType, vr::glUInt_t* pglTextureId, vr::CameraVideoStreamFrameHeader_t* pFrameHeader, uint32_t nFrameHeaderSize) = 0;
	virtual vr::EVRTrackedCameraError ReleaseVideoStreamTextureGL(vr::TrackedCameraHandle_t hTrackedCamera, vr::glUInt_t glTextureId) = 0;
};


static const char* const IVRTrackedCamera_005_Version = "IVRTrackedCamera_005";

class IVRTrackedCamera_005
{
public:
	virtual const char* GetCameraErrorNameFromEnum(vr::EVRTrackedCameraError eCameraError) = 0;
	virtual vr::EVRTrackedCameraError HasCamera(vr::TrackedDeviceIndex_t nDeviceIndex, bool* pHasCamera) = 0;
	virtual vr::EVRTrackedCameraError GetCameraFrameSize(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, uint32_t* pnWidth, uint32_t* pnHeight, uint32_t* pnFrameBufferSize) = 0;
	virtual vr::EVRTrackedCameraError GetCameraIntrinsics(vr::TrackedDeviceIndex_t nDeviceIndex, uint32_t nCameraIndex, vr::EVRTrackedCameraFrameType eFrameType, vr::HmdVector2_t* pFocalLength, vr::HmdVector2_t* pCenter) = 0;
	virtual vr::EVRTrackedCameraError GetCameraProjection(vr::TrackedDeviceIndex_t nDeviceIndex, uint32_t nCameraIndex, vr::EVRTrackedCameraFrameType eFrameType, float flZNear, float flZFar, vr::HmdMatrix44_t* pProjection) = 0;
	virtual vr::EVRTrackedCameraError AcquireVideoStreamingService(vr::TrackedDeviceIndex_t nDeviceIndex, vr::TrackedCameraHandle_t* pHandle) = 0;
	virtual vr::EVRTrackedCameraError ReleaseVideoStreamingService(vr::TrackedCameraHandle_t hTrackedCamera) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamFrameBuffer(vr::TrackedCameraHandle_t hTrackedCamera, vr::EVRTrackedCameraFrameType eFrameType, void* pFrameBuffer, uint32_t nFrameBufferSize, vr::CameraVideoStreamFrameHeader_t* pFrameHeader, uint32_t nFrameHeaderSize) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamTextureSize(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, vr::VRTextureBounds_t* pTextureBounds, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamTextureD3D11(vr::TrackedCameraHandle_t hTrackedCamera, vr::EVRTrackedCameraFrameType eFrameType, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView, vr::CameraVideoStreamFrameHeader_t* pFrameHeader, uint32_t nFrameHeaderSize) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamTextureGL(vr::TrackedCameraHandle_t hTrackedCamera, vr::EVRTrackedCameraFrameType eFrameType, vr::glUInt_t* pglTextureId, vr::CameraVideoStreamFrameHeader_t* pFrameHeader, uint32_t nFrameHeaderSize) = 0;
	virtual vr::EVRTrackedCameraError ReleaseVideoStreamTextureGL(vr::TrackedCameraHandle_t hTrackedCamera, vr::glUInt_t glTextureId) = 0;
};


static const char* const IVRTrackedCamera_006_Version = "IVRTrackedCamera_006";

class IVRTrackedCamera_006
{
public:
	virtual const char* GetCameraErrorNameFromEnum(vr::EVRTrackedCameraError eCameraError) = 0;
	virtual vr::EVRTrackedCameraError HasCamera(vr::TrackedDeviceIndex_t nDeviceIndex, bool* pHasCamera) = 0;
	virtual vr::EVRTrackedCameraError GetCameraFrameSize(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, uint32_t* pnWidth, uint32_t* pnHeight, uint32_t* pnFrameBufferSize) = 0;
	virtual vr::EVRTrackedCameraError GetCameraIntrinsics(vr::TrackedDeviceIndex_t nDeviceIndex, uint32_t nCameraIndex, vr::EVRTrackedCameraFrameType eFrameType, vr::HmdVector2_t* pFocalLength, vr::HmdVector2_t* pCenter) = 0;
	virtual vr::EVRTrackedCameraError GetCameraProjection(vr::TrackedDeviceIndex_t nDeviceIndex, uint32_t nCameraIndex, vr::EVRTrackedCameraFrameType eFrameType, float flZNear, float flZFar, vr::HmdMatrix44_t* pProjection) = 0;
	virtual vr::EVRTrackedCameraError AcquireVideoStreamingService(vr::TrackedDeviceIndex_t nDeviceIndex, vr::TrackedCameraHandle_t* pHandle) = 0;
	virtual vr::EVRTrackedCameraError ReleaseVideoStreamingService(vr::TrackedCameraHandle_t hTrackedCamera) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamFrameBuffer(vr::TrackedCameraHandle_t hTrackedCamera, vr::EVRTrackedCameraFrameType eFrameType, void* pFrameBuffer, uint32_t nFrameBufferSize, vr::CameraVideoStreamFrameHeader_t* pFrameHeader, uint32_t nFrameHeaderSize) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamTextureSize(vr::TrackedDeviceIndex_t nDeviceIndex, vr::EVRTrackedCameraFrameType eFrameType, vr::VRTextureBounds_t* pTextureBounds, uint32_t* pnWidth, uint32_t* pnHeight) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamTextureD3D11(vr::TrackedCameraHandle_t hTrackedCamera, vr::EVRTrackedCameraFrameType eFrameType, void* pD3D11DeviceOrResource, void** ppD3D11ShaderResourceView, vr::CameraVideoStreamFrameHeader_t* pFrameHeader, uint32_t nFrameHeaderSize) = 0;
	virtual vr::EVRTrackedCameraError GetVideoStreamTextureGL(vr::TrackedCameraHandle_t hTrackedCamera, vr::EVRTrackedCameraFrameType eFrameType, vr::glUInt_t* pglTextureId, vr::CameraVideoStreamFrameHeader_t* pFrameHeader, uint32_t nFrameHeaderSize) = 0;
	virtual vr::EVRTrackedCameraError ReleaseVideoStreamTextureGL(vr::TrackedCameraHandle_t hTrackedCamera, vr::glUInt_t glTextureId) = 0;
	virtual void SetCameraTrackingSpace(vr::ETrackingUniverseOrigin eUniverse) = 0;
	virtual vr::ETrackingUniverseOrigin GetCameraTrackingSpace() = 0;
};

} // namespace vr
