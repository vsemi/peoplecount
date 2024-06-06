#include "StreamBuffer.h"

#include <stdio.h>
#include <iostream>
#include <math.h>
#include "thread_safe.h"

using namespace std;
using namespace cv;

StreamBuffer::StreamBuffer() {
}
void StreamBuffer::update_current_frame(Mat* m) {
	synchronized(m_mutex_frame) {
		m_current_frame = m->clone();
	}
}
Mat StreamBuffer::get_current_frame() {
	Mat m_temp;
	if (m_current_frame.empty() || m_current_frame.rows == 0 || m_current_frame.cols == 0) {
		return m_temp;
	}
	else {
		synchronized(m_mutex_frame) {
			m_temp = m_current_frame.clone();
			m_current_frame = Mat();
		}
		return m_temp;
	}
}
