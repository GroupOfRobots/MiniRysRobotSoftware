#include "minirys_ros2/helpers/TimeMeasure.hpp"

#include <fstream>
#include <iostream>

TimeMeasure::TimeMeasure(int histogramBins, uint64_t binWidth, int topValuesCount) {
	this->histogramBins = histogramBins;
	this->histogram.resize(histogramBins);
	this->histogramBinWidth = binWidth;
	this->histogramBinMax = 0;
	this->histogramOverflows = 0;

	this->topValuesCount = topValuesCount;
	this->topValues.resize(topValuesCount);
	this->topValuesIndices.resize(topValuesCount);
	this->index = 0;
}

void TimeMeasure::add(uint64_t ns) {
	// Update histogram data
	auto bi = static_cast<int>((ns + this->histogramBinWidth / 2) / this->histogramBinWidth);
	if (bi >= this->histogramBins) {
		std::cerr << "[TimeMeasure] value too high! cropping to last bin" << std::endl;
		bi = this->histogramBins;
		++this->histogramOverflows;
	}

	++this->histogram[bi];
	if (bi >= this->histogramBinMax) {
		this->histogramBinMax = bi + 1;
	}

	// update topn. index 0-this->topValuesCount corresponds to top1-topn.
	for (int i = 0; i < this->topValuesCount; ++i) {
		if (ns < this->topValues[i]) {
			continue;
		}
		// shift backward from i index.
		for (int j = this->topValuesCount - 1; j > i; --j) {
			this->topValues[j] = this->topValues[j - 1];
			this->topValuesIndices[j] = this->topValuesIndices[j - 1];
		}
		// insert new data to i index;
		this->topValues[i] = ns;
		this->topValuesIndices[i] = this->index;
		break;
	}

	this->index++;
}

void TimeMeasure::saveHistCSV(std::string filename) {
	std::ofstream histogramOutput(filename);
	if (!histogramOutput) {
		std::cerr << "failed to open " << filename << "." << std::endl;
		return;
	}
	histogramOutput << "latency,count" << std::endl;
	// Save overflows as "-1"
	histogramOutput << "-1," << this->histogramOverflows << std::endl;
	for (int i = 0; i < this->histogramBinMax; i++) {
		histogramOutput << i << "," << this->histogram[i] << std::endl;
	}
	histogramOutput.close();
}

void TimeMeasure::saveTopNCSV(std::string filename) {
	std::ofstream topNOutput(filename);
	if (!topNOutput) {
		std::cerr << "failed to open " << filename << "." << std::endl;
		return;
	}
	topNOutput << "latency,index" << std::endl;
	for (int i = 0; i < this->topValuesCount; i++) {
		topNOutput << this->topValues[i] << "," << this->topValuesIndices[i] << std::endl;
	}
	topNOutput.close();
}
