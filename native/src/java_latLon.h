#ifndef _JAVA_LATLON_H
#define _JAVA_LATLON_H
#include "commonOsmAndCore.h"
#include "jni.h"

jclass jclass_LatLon;
jmethodID jmethod_LatLonClass_init;
jfieldID LatLon_longitude;
jfieldID LatLon_latitude;

jobject convertLatLonToJava(JNIEnv* env, double lat, double lon) {
	jclass_LatLon = env->FindClass("net/osmand/data/LatLon");
	jmethod_LatLonClass_init = env->GetMethodID(jclass_LatLon, "<init>", "(DD)V");
	jobject jLatLon = env->NewObject(jclass_LatLon, jmethod_LatLonClass_init, lat, lon);
	return jLatLon;
}

LatLon getLatLonFromJava(JNIEnv* env, jobject jLatLon) {
	jclass_LatLon = env->FindClass("net/osmand/data/LatLon");
	jfieldID jfield_LatLon_lat = env->GetFieldID(jclass_LatLon, "latitude", "D");
	jfieldID jfield_LatLon_lon = env->GetFieldID(jclass_LatLon, "longitude", "D");
	double lat = env->GetDoubleField(jLatLon, jfield_LatLon_lat);
	double lon = env->GetDoubleField(jLatLon, jfield_LatLon_lon);
	LatLon latLon(lat, lon);
	return latLon;
}

#endif