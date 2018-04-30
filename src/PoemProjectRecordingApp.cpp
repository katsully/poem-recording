#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Kinect2.h"
#include "cinder/params/Params.h"
#include "cinder/osc/Osc.h"
#include <fstream>
#include <ctime>
//#include "kat_decision_tree.h"

using namespace ci;
using namespace ci::app;
using namespace std;

#define USE_UDP 1

#if USE_UDP
using Sender = osc::SenderUdp;
using Receiver = osc::ReceiverUdp;
using protocol = asio::ip::udp;
#else
using Sender = osc::SenderTcp;
using Receiver = osc::ReceiverTcp;
using protocol = asio::ip::tcp;
#endif

// TODO: this should be in a json file (along with the vertex thresholds)
const std::string destinationHost = "127.0.0.1";
const uint16_t destinationPort = 8005;

class PoemProjectRecordingApp : public App {
public:
	PoemProjectRecordingApp();
	void setup() override;
	void update() override;
	void draw() override;
	void shutdown();

	Sender mSender;
	Receiver mReceiver;

private:
	Kinect2::DeviceRef mDevice;
	Kinect2::BodyFrame mBodyFrame;
	ci::Channel8uRef mChannelBodyIndex;
	ci::Channel16uRef mChannelDepth;

	params::InterfaceGlRef mParams;
	bool mDrawBackground = false;
	bool mDrawSkeleton = false;
	bool mDrawPentagon = true;

	// math
	vec3 numA1;
	vec3 numA2;
	vec3 numA3;
	vec3 numA4;
	vec3 numA5;

	vec3 numB1;
	vec3 numB2;
	vec3 numB3;
	vec3 numB4;
	vec3 numB5;

	vec3 numC1;
	vec3 numC2;
	vec3 numC3;
	vec3 numC4;
	vec3 numC5;

	// distance between two people
	float dist1 = 0.0;
	float dist2 = 0.0;
	float dist3 = 0.0;
	float dist4 = 0.0;
	float dist5 = 0.0;

	// points for drawing pentagons
	vec2 point1;
	vec2 point2;
	vec2 point3;
	vec2 point4;
	vec2 point5;

	bool mFullScreen = false;
	bool mShowParams = false;

	ofstream myfile;
	bool mRecording = false;
	int prevState = 0;
	bool consistentReading = true;
	int touchingCounter = 0;
};

PoemProjectRecordingApp::PoemProjectRecordingApp() : App(), mReceiver(8010), mSender(8000, destinationHost, destinationPort) {
	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectBodyEventHandler([&](const Kinect2::BodyFrame frame) {
		mBodyFrame = frame;
	});
	mDevice->connectBodyIndexEventHandler([&](const Kinect2::BodyIndexFrame frame) {
		mChannelBodyIndex = frame.getChannel();
	});
	mDevice->connectDepthEventHandler([&](const Kinect2::DepthFrame frame) {
		mChannelDepth = frame.getChannel();
	});
}

void PoemProjectRecordingApp::setup()
{
	setFullScreen(mFullScreen);

	// create a parameter interface and name it
	mParams = params::InterfaceGl::create(getWindow(), "App Parameters", toPixels(ivec2(200, 400)));

	// setup parameters
	mParams->addParam("Draw Background", &mDrawBackground);
	mParams->addParam("Draw Skeleton", &mDrawSkeleton);
	mParams->addParam("Draw Pentagon", &mDrawPentagon);

	mParams->addSeparator();

	mParams->addParam("Distance 1", &dist1);
	mParams->addParam("Distance 2", &dist2);
	mParams->addParam("Distance 3", &dist3);
	mParams->addParam("Distance 4", &dist4);
	mParams->addParam("Distance 5", &dist5);

	mParams->addSeparator();

	mParams->addParam("Full Screen", &mFullScreen).updateFn([this] { setFullScreen(mFullScreen); });
	mParams->addParam("Show Params", &mShowParams).key("p");

	// TODO - param button to quit

	std::time_t t = std::time(0);	// get time now
	std::tm* now = std::localtime(&t);

}

void PoemProjectRecordingApp::update()
{
}

void PoemProjectRecordingApp::draw()
{
	const gl::ScopedViewport scopedViewport(ivec2(0), getWindowSize());
	const gl::ScopedMatrices scopedMatrices;
	const gl::ScopedBlendAlpha scopedBlendAlpha;
	gl::setMatricesWindow(getWindowSize());
	gl::clear();
	gl::color(ColorAf::white());
	gl::disableDepthRead();
	gl::disableDepthWrite();

	if (mChannelDepth && mDrawBackground) {
		gl::enable(GL_TEXTURE_2D);
		const gl::TextureRef tex = gl::Texture::create(*Kinect2::channel16To8(mChannelDepth));
		gl::draw(tex, tex->getBounds(), Rectf(getWindowBounds()));
	}

	if (mChannelBodyIndex) {
		gl::enable(GL_TEXTURE_2D);

		gl::pushMatrices();
		gl::scale(vec2(getWindowSize()) / vec2(mChannelBodyIndex->getSize()));
		gl::disable(GL_TEXTURE_2D);
		//int bodyCounter = 0;
		for (const Kinect2::Body &body : mBodyFrame.getBodies()) {
			if (body.isTracked()) {
				gl::color(ColorAf::white());

				auto map = body.getJointMap();

				numA1 = (map.at(JointType_Head).getPosition() + map.at(JointType_SpineShoulder).getPosition()) / vec3(2);
				numA2 = (map.at(JointType_ShoulderRight).getPosition() + map.at(JointType_ElbowRight).getPosition() + map.at(JointType_WristRight).getPosition() + map.at(JointType_HandRight).getPosition()) / vec3(4);
				numA3 = (map.at(JointType_ShoulderLeft).getPosition() + map.at(JointType_ElbowLeft).getPosition() + map.at(JointType_WristLeft).getPosition() + map.at(JointType_HandLeft).getPosition()) / vec3(4);
				numA4 = (map.at(JointType_HipRight).getPosition() + map.at(JointType_KneeRight).getPosition() + map.at(JointType_AnkleRight).getPosition() + map.at(JointType_FootRight).getPosition()) / vec3(4);
				numA5 = (map.at(JointType_HipLeft).getPosition() + map.at(JointType_KneeLeft).getPosition() + map.at(JointType_AnkleLeft).getPosition() + map.at(JointType_FootLeft).getPosition()) / vec3(4);


				if (mRecording) {
					myfile << to_string(dist1) + ",";
					myfile << to_string(dist2) + ",";
					myfile << to_string(dist3) + ",";
					myfile << to_string(dist4) + ",";
					myfile << to_string(dist5) + ",";
					myfile << "\n";
				}

				// draw skeletons
				for (const auto& joint : map) {
					if (joint.second.getTrackingState() == TrackingState::TrackingState_Tracked) {
						vec2 pos(mDevice->mapCameraToDepth(joint.second.getPosition()));
						if (mDrawSkeleton) {
							gl::drawSolidCircle(pos, 5.0f, 32);
							vec2 parent(mDevice->mapCameraToDepth(map.at(joint.second.getParentJoint()).getPosition()));
							gl::drawLine(pos, parent);
						}
					}
				}
				if (mDrawPentagon) {
					// calculate positions which we will use to calculate the five vertices of the pentagons
					vec2 headPos(mDevice->mapCameraToDepth(map.at(JointType_Head).getPosition()));
					vec2 shoulderPos(mDevice->mapCameraToDepth(map.at(JointType_SpineShoulder).getPosition()));

					vec2 shoulderRPos(mDevice->mapCameraToDepth(map.at(JointType_ShoulderRight).getPosition()));
					vec2 elbowRPos(mDevice->mapCameraToDepth(map.at(JointType_ElbowRight).getPosition()));
					vec2 wristRPos(mDevice->mapCameraToDepth(map.at(JointType_WristRight).getPosition()));
					vec2 handRPos(mDevice->mapCameraToDepth(map.at(JointType_HandRight).getPosition()));

					vec2 shoulderLPos(mDevice->mapCameraToDepth(map.at(JointType_ShoulderLeft).getPosition()));
					vec2 elbowLPos(mDevice->mapCameraToDepth(map.at(JointType_ElbowLeft).getPosition()));
					vec2 wristLPos(mDevice->mapCameraToDepth(map.at(JointType_WristLeft).getPosition()));
					vec2 handLPos(mDevice->mapCameraToDepth(map.at(JointType_HandLeft).getPosition()));

					vec2 hipRPos(mDevice->mapCameraToDepth(map.at(JointType_HipRight).getPosition()));
					vec2 kneeRPos(mDevice->mapCameraToDepth(map.at(JointType_KneeRight).getPosition()));
					vec2 ankleRPos(mDevice->mapCameraToDepth(map.at(JointType_AnkleRight).getPosition()));
					vec2 footRPos(mDevice->mapCameraToDepth(map.at(JointType_FootRight).getPosition()));

					vec2 hipLPos(mDevice->mapCameraToDepth(map.at(JointType_HipLeft).getPosition()));
					vec2 kneeLPos(mDevice->mapCameraToDepth(map.at(JointType_KneeLeft).getPosition()));
					vec2 ankleLPos(mDevice->mapCameraToDepth(map.at(JointType_AnkleLeft).getPosition()));
					vec2 footLPos(mDevice->mapCameraToDepth(map.at(JointType_FootLeft).getPosition()));

					gl::lineWidth(5.0f);

					// draw pentagon for 1st person
					// TODO: for if statements, only put colors, then have one block of code for points, circles, and lines, don't need pointA1, pointB1, pointC1, only point1
					if (bodyCounter == 0) {
						gl::color(1, 0, 0);
					}
					// draw pentagon for 2nd person
					else if (bodyCounter == 1) {
						gl::color(0, 1, 0);
					}

					// draw pentagon for 3rd person
					else if (bodyCounter == 2) {
						gl::color(0, 0, 1);
					}
					point1 = (headPos + shoulderPos) / vec2(2);
					gl::drawSolidCircle(point1, 5.0f, 32);


					point2 = (shoulderRPos + elbowRPos + wristRPos + handRPos) / vec2(4);
					gl::drawSolidCircle(point2, 5.0f, 32);


					point3 = (shoulderLPos + elbowLPos + wristLPos + handLPos) / vec2(4);
					gl::drawSolidCircle(point3, 5.0f, 32);

					point4 = (hipRPos + kneeRPos + ankleRPos + footRPos) / vec2(4);
					gl::drawSolidCircle(point4, 5.0f, 32);

					point5 = (hipLPos + kneeLPos + ankleLPos + footLPos) / vec2(4);
					gl::drawSolidCircle(point5, 5.0f, 32);

					gl::drawLine(point1, point2);
					gl::drawLine(point2, point4);
					gl::drawLine(point4, point5);
					gl::drawLine(point5, point3);
					gl::drawLine(point3, point1);
				}
				drawHand(body.getHandLeft(), mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HandLeft).getPosition()));
				drawHand(body.getHandRight(), mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HandRight).getPosition()));

				bodyCounter++;
			}
		}

		// if the sensors say they are touching
		if (mTouching) {

			console() << "touching" << endl;

			int counterBod = 0;
			for (const Kinect2::Body &body : mBodyFrame.getBodies()) {
				if (body.isTracked()) {
					counterBod += 1;
				}
			}

			console() << "number of bodies " << counterBod << endl;

			// if there are only 1 or 0 bodies being detected, most likely it lost tracking which happens during a hug
			if (counterBod < 2) {
				console() << "0 or 1 bodies" << endl;
				state = 0;
			}

			// compare person A & B
			else if (counterBod == 2) {
				// calculate distances
				dist1 = sqrt(math<float>::pow(numA1.x - numB1.x, 2) + math<float>::pow(numA1.y - numB1.y, 2) + math<float>::pow(numA1.z - numB1.z, 2));
				dist2 = sqrt(math<float>::pow(numA2.x - numB2.x, 2) + math<float>::pow(numA2.y - numB2.y, 2) + math<float>::pow(numA2.z - numB2.z, 2));
				dist3 = sqrt(math<float>::pow(numA3.x - numB3.x, 2) + math<float>::pow(numA3.y - numB3.y, 2) + math<float>::pow(numA3.z - numB3.z, 2));
				dist4 = sqrt(math<float>::pow(numA4.x - numB4.x, 2) + math<float>::pow(numA4.y - numB4.y, 2) + math<float>::pow(numA4.z - numB4.z, 2));
				dist5 = sqrt(math<float>::pow(numA5.x - numB5.x, 2) + math<float>::pow(numA5.y - numB5.y, 2) + math<float>::pow(numA5.z - numB5.z, 2));



				vector<double> distances;
				distances.push_back(dist1);
				distances.push_back(dist2);
				distances.push_back(dist3);
				distances.push_back(dist4);
				distances.push_back(dist5);

				/*console() << "first in list " <<  distances[0] << endl;
				console() << "last in list " << distances[4] << endl;
				console() << "decision tree " << kat_decision_tree(distances) << endl;*/

				// return if hugging
				if (kat_decision_tree(distances) == 0) {
					state = 0;
				}
				// return if hand holding
				else if (kat_decision_tree(distances) == 1) {
					state = 1;
				}
				distances.clear();
			}
			// if there are 3 (or more) people being detected compare A&B and B&C
			// doesn't make sense to check A&C since B is inbetween them
			else {

				console() << "three bodies" << endl;
				if (mTouchingAB == true) {
					// A&B
					// calculate distances
					dist1 = sqrt(math<float>::pow(numA1.x - numB1.x, 2) + math<float>::pow(numA1.y - numB1.y, 2) + math<float>::pow(numA1.z - numB1.z, 2));
					dist2 = sqrt(math<float>::pow(numA2.x - numB2.x, 2) + math<float>::pow(numA2.y - numB2.y, 2) + math<float>::pow(numA2.z - numB2.z, 2));
					dist3 = sqrt(math<float>::pow(numA3.x - numB3.x, 2) + math<float>::pow(numA3.y - numB3.y, 2) + math<float>::pow(numA3.z - numB3.z, 2));
					dist4 = sqrt(math<float>::pow(numA4.x - numB4.x, 2) + math<float>::pow(numA4.y - numB4.y, 2) + math<float>::pow(numA4.z - numB4.z, 2));
					dist5 = sqrt(math<float>::pow(numA5.x - numB5.x, 2) + math<float>::pow(numA5.y - numB5.y, 2) + math<float>::pow(numA5.z - numB5.z, 2));

					vector<double> distances;
					distances.push_back(dist1);
					distances.push_back(dist2);
					distances.push_back(dist3);
					distances.push_back(dist4);
					distances.push_back(dist5);

					//console() << "START~~~" << endl;
					//console() << "dist 1 " << dist1 << endl;
					//console() << "dist 2 " << dist2 << endl;
					//console() << "dist 3 " << dist3 << endl;
					//console() << "dist 4 " << dist4 << endl;
					//console() << "dist 5 " << dist5 << endl;

					// return if hugging
					if (kat_decision_tree(distances) == 0) {
						state = 0;
					}
					// return if hand holding
					else if (kat_decision_tree(distances) == 1) {
						state = 1;
					}
				}
				else if (mTouchingBC == true) {
					// B&C
					// calculate distances
					dist1 = sqrt(math<float>::pow(numC1.x - numB1.x, 2) + math<float>::pow(numC1.y - numB1.y, 2) + math<float>::pow(numC1.z - numB1.z, 2));
					dist2 = sqrt(math<float>::pow(numC2.x - numB2.x, 2) + math<float>::pow(numC2.y - numB2.y, 2) + math<float>::pow(numC2.z - numB2.z, 2));
					dist3 = sqrt(math<float>::pow(numC3.x - numB3.x, 2) + math<float>::pow(numC3.y - numB3.y, 2) + math<float>::pow(numC3.z - numB3.z, 2));
					dist4 = sqrt(math<float>::pow(numC4.x - numB4.x, 2) + math<float>::pow(numC4.y - numB4.y, 2) + math<float>::pow(numC4.z - numB4.z, 2));
					dist5 = sqrt(math<float>::pow(numC5.x - numB5.x, 2) + math<float>::pow(numC5.y - numB5.y, 2) + math<float>::pow(numC5.z - numB5.z, 2));

					vector<double> distances;
					distances.push_back(dist1);
					distances.push_back(dist2);
					distances.push_back(dist3);
					distances.push_back(dist4);
					distances.push_back(dist5);

					// return if hugging
					if (kat_decision_tree(distances) == 0) {
						state = 0;
					}
					// return if hand holding
					else if (kat_decision_tree(distances) == 1) {
						state = 1;
					}
				}
			}

			if (state == 0 && prevState == 0) {
				touchingCounter += 1;
				if (touchingCounter > 25) {
					osc::Message msg("/Case_2");
					mSender.send(msg);
					mTouching = false;
					mTouchingAB = false;
					mTouchingBC = false;
					console() << "CASE TWO" << endl;
				}
			}
			else if (state == 1 && prevState == 1) {
				touchingCounter += 1;
				if (touchingCounter > 25) {
					osc::Message msg("/Case_1");
					mSender.send(msg);
					mTouching = false;
					mTouchingAB = false;
					mTouchingBC = false;
					console() << "CASE FFREAKING ONE" << endl;
				}
			}
			else {
				prevState = state;
				touchingCounter = 0;
			}

		}
	}

	// draw parameters interface
	if (mShowParams) {
		mParams->draw();
	}
}

void PoemProjectRecordingApp::shutdown() {
	mDevice->stop();
	myfile.close();
}

CINDER_APP(PoemProjectRecordingApp, RendererGl)