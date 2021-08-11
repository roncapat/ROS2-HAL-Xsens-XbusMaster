#include "libcmt/cmt2.h"
#include <cstdlib>
#include <cstdio>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <iostream>
#include <string>
#include <exception>

// sudo rfcomm bind xmaster 00:12:F3:0B:68:87

using namespace xsens;
namespace py = pybind11;

enum Mode {
	CalibratedData = 0,
	OrientationEstimate = 1,
	All = 2
};

class PyHALXsens{
	private:
		Cmt2s serial;
		Message msg, reply;
		int mode;
		int slaves_n = 0;
		int frequency = 0;
		bool silent = true;
	public:
		PyHALXsens(){}

		inline int get_slaves_number() const {return slaves_n;}

		void ensureReply(Message &msg, Message &reply, uint8_t reply_type, bool silent=true){
			while(serial.waitForMessage(&reply, reply_type, 1000,  1) != XRV_OK){
				serial.writeMessage(&msg);
				if (not silent) std::cout << "*" << std::flush;
			}
			if (not silent) std::cout << std::endl;
		}

		void initMsg(Message &msg, uint8_t msg_id, uint8_t bus_id = 0 /* broadcast */){
			msg.setMessageId(msg_id);
			msg.setBusId(bus_id);
			msg.resizeData(0);
		}

		void printMessage(Message &reply, bool skip=false){
			if (skip) return;
			std::cout << "DEVICE:     " << (unsigned int) reply.getBusId() << std::endl;
			std::cout << "MESSAGE:    " << std::hex << std::uppercase 
										<< "0x" << (unsigned int) reply.getMessageId() 
										<< std::dec << std::nouppercase << std::endl;
			std::cout << "LENGTH:     " << std::dec << (unsigned int) reply.getDataSize() << std::endl;
			if (reply.getDataSize() > 0){
			std::cout << "HEX DATA:   ";
				for (int i = 0; i < reply.getDataSize(); i++){
					std::cout << std::hex << std::uppercase << (int)reply.getDataByte(i) << " " << std::flush;
				}
				std::cout << std::dec << std::nouppercase << std::endl << std::endl;
			}
		}

		bool init(int mode, std::string port){

		    if (serial.open(port.c_str(), B115200) != XRV_OK){
		    	std::cout << "Failed to open " << port << std::endl;
		    	return false;
		    }

			initMsg(msg, CMT_MID_REQDID, 255);
			ensureReply(msg, reply, CMT_MID_DEVICEID, silent);
			printMessage(reply, silent);

			initMsg(msg, CMT_MID_INITBUS, 255);
			ensureReply(msg, reply, CMT_MID_INITBUSRESULTS, silent);
			printMessage(reply,silent);

			slaves_n = reply.getDataSize()/4;
			std::cout << "XBus Master identified " << slaves_n << " devices on the bus." << std::endl;

			initMsg(msg, CMT_MID_REQPERIOD, 255);
			ensureReply(msg, reply, CMT_MID_REQPERIODACK, silent);
			printMessage(reply,silent);

			frequency = 115200/reply.getDataShort();
			std::cout << "XBus Master sampling period is " << frequency << " Hz." << std::endl;

			initMsg(msg, CMT_MID_GOTOCONFIG, 255);
			ensureReply(msg, reply, CMT_MID_GOTOCONFIGACK, silent);
			printMessage(reply,silent);
			std::cout << "XBus Master has entered Config Mode" << std::endl;

			for (int slave_id = 1; slave_id<=slaves_n; slave_id++){
				initMsg(msg, CMT_MID_RESETORIENTATION, slave_id);
				msg.setDataShort(CMT_RESETORIENTATION_ALIGN);
				ensureReply(msg, reply, CMT_MID_RESETORIENTATIONACK, silent);
				printMessage(reply,silent);
				std::cout << "\tXBus Slave "<< slave_id << " has reset its orientation" << std::endl;

				if (mode == Mode::CalibratedData){
					initMsg(msg, CMT_MID_SETOUTPUTMODE, slave_id);
					msg.setDataShort(CMT_OUTPUTMODE_CALIB);
					ensureReply(msg, reply, CMT_MID_SETOUTPUTMODEACK, silent);
					printMessage(reply,silent);
				
					initMsg(msg, CMT_MID_SETOUTPUTSETTINGS, slave_id);
					msg.setDataLong(CMT_OUTPUTSETTINGS_CALIBMODE_ACCGYRMAG | CMT_OUTPUTSETTINGS_TIMESTAMP_NONE);
					ensureReply(msg, reply, CMT_MID_SETOUTPUTSETTINGSACK, silent);
					printMessage(reply,silent);

				} else if (mode == Mode::OrientationEstimate){
					initMsg(msg, CMT_MID_SETOUTPUTMODE, slave_id);
					msg.setDataShort(CMT_OUTPUTMODE_ORIENT);
					ensureReply(msg, reply, CMT_MID_SETOUTPUTMODEACK, silent);
					printMessage(reply,silent);
				
					initMsg(msg, CMT_MID_SETOUTPUTSETTINGS, slave_id);
					msg.setDataLong(CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX | CMT_OUTPUTSETTINGS_TIMESTAMP_NONE);
					ensureReply(msg, reply, CMT_MID_SETOUTPUTSETTINGSACK, silent);
					printMessage(reply,silent);

				} else if (mode == Mode::All){
					initMsg(msg, CMT_MID_SETOUTPUTMODE, slave_id);
					msg.setDataShort(CMT_OUTPUTMODE_ORIENT | CMT_OUTPUTMODE_CALIB);
					ensureReply(msg, reply, CMT_MID_SETOUTPUTMODEACK, silent);
					printMessage(reply,silent);
				
					initMsg(msg, CMT_MID_SETOUTPUTSETTINGS, slave_id);
					msg.setDataLong(CMT_OUTPUTSETTINGS_CALIBMODE_ACCGYRMAG | CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX | CMT_OUTPUTSETTINGS_TIMESTAMP_NONE);
					ensureReply(msg, reply, CMT_MID_SETOUTPUTSETTINGSACK, silent);
					printMessage(reply,silent);
				} else {
					std::cout << "Unrecognised mode" << std::endl;
					return false;
				}
				std::cout << "\tXBus Slave "<< slave_id << " has accepted output configuration" << std::endl;
			}

			initMsg(msg, CMT_MID_GOTOMEASUREMENT, 255);
			ensureReply(msg, reply, CMT_MID_GOTOMEASUREMENTACK, silent);
			printMessage(reply,silent);
			std::cout << "XBus Master started to stream slaves data" << std::endl;
			return true;
		}

		~PyHALXsens(){}

		bool get_gyro_acc(py::array_t<float, py::array::c_style | py::array::forcecast> rotvel,
     					  py::array_t<float, py::array::c_style | py::array::forcecast> linacc){
     		if ((mode != Mode::CalibratedData) and (mode != Mode::All)) return false;
			auto rotvel_ptr = (float*)rotvel.request().ptr;
			auto linacc_ptr = (float*)linacc.request().ptr;
			//auto gil_release = py::gil_scoped_release();
			auto ret = serial.waitForMessage(&reply, 0, 0, 1);
			if (ret != XRV_OK) return false;
			for (int slave_id = 1; slave_id<=slaves_n; slave_id++){
				rotvel_ptr[3*(slave_id-1) + 0] = reply.getDataFloat(2 + 24*(slave_id-1) + 3*4);      // [rad/s]
				rotvel_ptr[3*(slave_id-1) + 1] = reply.getDataFloat(2 + 24*(slave_id-1) + 4*4);      // [rad/s]
				rotvel_ptr[3*(slave_id-1) + 2] = reply.getDataFloat(2 + 24*(slave_id-1) + 5*4);      // [rad/s]
				linacc_ptr[3*(slave_id-1) + 0] = reply.getDataFloat(2 + 24*(slave_id-1) + 0*4);        // [m/s2]
				linacc_ptr[3*(slave_id-1) + 1] = reply.getDataFloat(2 + 24*(slave_id-1) + 1*4);        // [m/s2]
				linacc_ptr[3*(slave_id-1) + 2] = reply.getDataFloat(2 + 24*(slave_id-1) + 2*4);        // [m/s2]
			}
			return true;
		}

		bool get_gyro_acc_mag(py::array_t<float, py::array::c_style | py::array::forcecast> rotvel,
     					  py::array_t<float, py::array::c_style | py::array::forcecast> linacc,
     					  py::array_t<float, py::array::c_style | py::array::forcecast> magfld){
     		if ((mode != Mode::CalibratedData) and (mode != Mode::All)) return false;
			auto rotvel_ptr = (float*)rotvel.request().ptr;
			auto linacc_ptr = (float*)linacc.request().ptr;
			auto magfld_ptr = (float*)magfld.request().ptr;
			//auto gil_release = py::gil_scoped_release();
			auto ret = serial.waitForMessage(&reply, 0, 0, 1);
			if (ret != XRV_OK) return false;
			for (int slave_id = 1; slave_id<=slaves_n; slave_id++){
				rotvel_ptr[3*(slave_id-1) + 0] = reply.getDataFloat(2 + 36*(slave_id-1) + 3*4);      // [rad/s]
				rotvel_ptr[3*(slave_id-1) + 1] = reply.getDataFloat(2 + 36*(slave_id-1) + 4*4);      // [rad/s]
				rotvel_ptr[3*(slave_id-1) + 2] = reply.getDataFloat(2 + 36*(slave_id-1) + 5*4);      // [rad/s]
				linacc_ptr[3*(slave_id-1) + 0] = reply.getDataFloat(2 + 36*(slave_id-1) + 0*4);        // [m/s2]
				linacc_ptr[3*(slave_id-1) + 1] = reply.getDataFloat(2 + 36*(slave_id-1) + 1*4);        // [m/s2]
				linacc_ptr[3*(slave_id-1) + 2] = reply.getDataFloat(2 + 36*(slave_id-1) + 2*4);        // [m/s2]
				magfld_ptr[3*(slave_id-1) + 0] = reply.getDataFloat(2 + 36*(slave_id-1) + 6*4);        // [arbitrary, normalized to earth field strength]
				magfld_ptr[3*(slave_id-1) + 1] = reply.getDataFloat(2 + 36*(slave_id-1) + 7*4);        // [arbitrary, normalized to earth field strength]
				magfld_ptr[3*(slave_id-1) + 2] = reply.getDataFloat(2 + 36*(slave_id-1) + 8*4);        // [arbitrary, normalized to earth field strength]
			}
			return true;
		}

		bool get_rot(py::array_t<float, py::array::c_style | py::array::forcecast> rotmat){
     		if ((mode != Mode::OrientationEstimate) and (mode != Mode::All)) return false;
			auto rotmat_ptr = (float*)rotmat.request().ptr;
			//auto gil_release = py::gil_scoped_release();
			auto ret = serial.waitForMessage(&reply, 0, 0, 1);
			if (ret != XRV_OK) return false;
			for (int slave_id = 1; slave_id<=slaves_n; slave_id++){
				if (mode != Mode::All){
					rotmat_ptr[9*(slave_id-1) + 0] = reply.getDataFloat(2 + 36*(slave_id-1) + 0*4);
					rotmat_ptr[9*(slave_id-1) + 1] = reply.getDataFloat(2 + 36*(slave_id-1) + 3*4);
					rotmat_ptr[9*(slave_id-1) + 2] = reply.getDataFloat(2 + 36*(slave_id-1) + 6*4);
					rotmat_ptr[9*(slave_id-1) + 3] = reply.getDataFloat(2 + 36*(slave_id-1) + 1*4);
					rotmat_ptr[9*(slave_id-1) + 4] = reply.getDataFloat(2 + 36*(slave_id-1) + 4*4);
					rotmat_ptr[9*(slave_id-1) + 5] = reply.getDataFloat(2 + 36*(slave_id-1) + 7*4);
					rotmat_ptr[9*(slave_id-1) + 6] = reply.getDataFloat(2 + 36*(slave_id-1) + 2*4);
					rotmat_ptr[9*(slave_id-1) + 7] = reply.getDataFloat(2 + 36*(slave_id-1) + 5*4);
					rotmat_ptr[9*(slave_id-1) + 8] = reply.getDataFloat(2 + 36*(slave_id-1) + 8*4);
				} else {
					rotmat_ptr[9*(slave_id-1) + 0] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 0*4);
					rotmat_ptr[9*(slave_id-1) + 1] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 3*4);
					rotmat_ptr[9*(slave_id-1) + 2] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 6*4);
					rotmat_ptr[9*(slave_id-1) + 3] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 1*4);
					rotmat_ptr[9*(slave_id-1) + 4] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 4*4);
					rotmat_ptr[9*(slave_id-1) + 5] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 7*4);
					rotmat_ptr[9*(slave_id-1) + 6] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 2*4);
					rotmat_ptr[9*(slave_id-1) + 7] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 5*4);
					rotmat_ptr[9*(slave_id-1) + 8] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 8*4);
				}
			}
			return true;
		}

		bool get_gyro_acc_mag_rot(py::array_t<float, py::array::c_style | py::array::forcecast> rotvel,
     					  py::array_t<float, py::array::c_style | py::array::forcecast> linacc,
     					  py::array_t<float, py::array::c_style | py::array::forcecast> magfld,
						  py::array_t<float, py::array::c_style | py::array::forcecast> rotmat){
     		if (mode != Mode::All) return false;
			auto rotvel_ptr = (float*)rotvel.request().ptr;
			auto linacc_ptr = (float*)linacc.request().ptr;
			auto magfld_ptr = (float*)magfld.request().ptr;
			auto rotmat_ptr = (float*)rotmat.request().ptr;
			//auto gil_release = py::gil_scoped_release();
			auto ret = serial.waitForMessage(&reply, 0, 0, 1);
			if (ret != XRV_OK) return false;
			for (int slave_id = 1; slave_id<=slaves_n; slave_id++){
				rotvel_ptr[3*(slave_id-1) + 0] = reply.getDataFloat(2 + 72*(slave_id-1) + 3*4);      // [rad/s]
				rotvel_ptr[3*(slave_id-1) + 1] = reply.getDataFloat(2 + 72*(slave_id-1) + 4*4);      // [rad/s]
				rotvel_ptr[3*(slave_id-1) + 2] = reply.getDataFloat(2 + 72*(slave_id-1) + 5*4);      // [rad/s]
				linacc_ptr[3*(slave_id-1) + 0] = reply.getDataFloat(2 + 72*(slave_id-1) + 0*4);        // [m/s2]
				linacc_ptr[3*(slave_id-1) + 1] = reply.getDataFloat(2 + 72*(slave_id-1) + 1*4);        // [m/s2]
				linacc_ptr[3*(slave_id-1) + 2] = reply.getDataFloat(2 + 72*(slave_id-1) + 2*4);        // [m/s2]
				magfld_ptr[3*(slave_id-1) + 0] = reply.getDataFloat(2 + 72*(slave_id-1) + 6*4);        // [arbitrary, normalized to earth field strength]
				magfld_ptr[3*(slave_id-1) + 1] = reply.getDataFloat(2 + 72*(slave_id-1) + 7*4);        // [arbitrary, normalized to earth field strength]
				magfld_ptr[3*(slave_id-1) + 2] = reply.getDataFloat(2 + 72*(slave_id-1) + 8*4);        // [arbitrary, normalized to earth field strength]
				rotmat_ptr[9*(slave_id-1) + 0] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 0*4);
				rotmat_ptr[9*(slave_id-1) + 1] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 3*4);
				rotmat_ptr[9*(slave_id-1) + 2] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 6*4);
				rotmat_ptr[9*(slave_id-1) + 3] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 1*4);
				rotmat_ptr[9*(slave_id-1) + 4] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 4*4);
				rotmat_ptr[9*(slave_id-1) + 5] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 7*4);
				rotmat_ptr[9*(slave_id-1) + 6] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 2*4);
				rotmat_ptr[9*(slave_id-1) + 7] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 5*4);
				rotmat_ptr[9*(slave_id-1) + 8] = reply.getDataFloat(2 + 72*(slave_id-1) + 36 + 8*4);
			}
			return true;
		}

		bool has_mag(){
			if ((mode != Mode::CalibratedData) or (mode != Mode::All)) return false;
			return true;
		}

		bool has_raw(){
			if ((mode != Mode::CalibratedData) or (mode != Mode::All)) return false;
			return true;
		}

		bool has_rot(){
			if ((mode != Mode::OrientationEstimate) or (mode != Mode::All)) return false;
			return true;
		}
};

PYBIND11_MODULE(xsens_XMB, m) {
    py::class_<PyHALXsens>(m, "HAL")
		.def(py::init<>())
		.def("init", &PyHALXsens::init)
		.def("get_gyro_acc", &PyHALXsens::get_gyro_acc)
		.def("get_gyro_acc_mag", &PyHALXsens::get_gyro_acc_mag)
		.def("get_gyro_acc_mag_rot", &PyHALXsens::get_gyro_acc_mag_rot)
		.def("get_rot", &PyHALXsens::get_rot)
		.def("has_raw", &PyHALXsens::has_raw)
		.def("has_mag", &PyHALXsens::has_mag)
		.def("has_rot", &PyHALXsens::has_rot)
		.def("get_slaves_number", &PyHALXsens::get_slaves_number);
	py::enum_<Mode>(m, "Mode")
		.value("CalibratedData", CalibratedData)
		.value("OrientationEstimate", OrientationEstimate)
		.value("All", All)
		.export_values();
}
