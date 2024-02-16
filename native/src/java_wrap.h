#ifndef _JAVA_WRAP_H
#define _JAVA_WRAP_H

#include "Logging.h"
#include "binaryRead.h"
#include "commonRendering.h"
#include "generalRouter.h"
#include "jni.h"

struct ResultJNIPublisher : ResultPublisher {
	JNIEnv* env;
	jobject o;
	jfieldID interruptedField;
	ResultJNIPublisher(jobject o, jfieldID interruptedField, JNIEnv* env)
		: env(env), o(o), interruptedField(interruptedField) {
	}

	bool isCancelled() {
		if (env != NULL && o != NULL) {
			return env->GetBooleanField(o, interruptedField);
		}
		return false;
	}
};

struct JNIRenderingContext : RenderingContext {
	jobject javaRenderingContext;
	JNIEnv* env;
	JNIRenderingContext() : javaRenderingContext(NULL) {
	}

	virtual SkBitmap* getCachedBitmap(const std::string& bitmapResource);
	virtual std::string getTranslatedString(const std::string& src);
	virtual std::string getReshapedString(const std::string& src);

	virtual bool interrupted();
	virtual ~JNIRenderingContext() {
	}
};

void pullFromJavaRenderingContext(JNIEnv* env, jobject jrc, JNIRenderingContext* rc);
void pushToJavaRenderingContext(JNIEnv* env, jobject jrc, JNIRenderingContext* rc);

jobject newGlobalRef(JNIEnv* env, jobject o) {
	return env->NewGlobalRef(o);
}

void throwNewException(JNIEnv* env, const char* msg) {
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "%s", msg);
	env->ThrowNew(env->FindClass("java/lang/Exception"), msg);
}

jclass findClass(JNIEnv* env, const char* className, bool mustHave = true) {
	jclass javaClass = env->FindClass(className);
	if (!javaClass && mustHave) throwNewException(env, (std::string("Failed to find class ") + className).c_str());
	return (jclass)newGlobalRef(env, javaClass);
}

jfieldID getFid(JNIEnv* env, jclass cls, const char* fieldName, const char* sig) {
	jfieldID jfield = env->GetFieldID(cls, fieldName, sig);
	if (!jfield)
		throwNewException(
			env, (std::string("Failed to find field ") + fieldName + std::string(" with signature ") + sig).c_str());
	return jfield;
}

std::string getStringField(JNIEnv* env, jobject o, jfieldID fid) {
	jstring jstr = (jstring)env->GetObjectField(o, fid);
	if (!jstr) {
		throwNewException(env, "Failed to get object from field");
		return std::string();
	}
	const char* utfBytes = env->GetStringUTFChars(jstr, NULL);
	// TODO: I'm not quite sure that if real unicode will happen here, this will work as expected
	std::string result(utfBytes);
	env->ReleaseStringUTFChars(jstr, utfBytes);
	env->DeleteLocalRef(jstr);
	return result;
}

std::string getString(JNIEnv* env, jstring jstr) {
	if (!jstr) {
		throwNewException(env, "NULL jstring passed in");
		return std::string();
	}
	const char* utfBytes = env->GetStringUTFChars(jstr, NULL);
	// TODO: I'm not quite sure that if real unicode will happen here, this will work as expected
	std::string result(utfBytes);
	env->ReleaseStringUTFChars(jstr, utfBytes);
	return result;
}

std::string getStringMethod(JNIEnv* env, jobject o, jmethodID fid) {
	jstring js = (jstring)env->CallObjectMethod(o, fid);
	std::string s = getString(env, js);
	env->DeleteLocalRef(js);
	return s;
}

std::string getStringMethod(JNIEnv* env, jobject o, jmethodID fid, int i) {
	jstring js = (jstring)env->CallObjectMethod(o, fid, i);
	std::string s = getString(env, js);
	env->DeleteLocalRef(js);
	return s;
}
jobject convertTransportRouteToJava(JNIEnv* ienv, SHARED_PTR<TransportRoute>& route);
RouteAttributeExpression convertExpressionFromJava(JNIEnv* ienv, jobject jExpression);

void clearDirectionPointFromRouteResult(SHARED_PTR<RouteSegmentResult> r );

#endif
