#pragma once

#include <string>
#include <vector>

/**
 * This class saves the histogram of a time measurements and the N highest measurements
 */
class TimeMeasure {
public:
	/**
	 * Initialize a time measurement helper.
	 *
	 * @param histogramBins number of bins in the histogram
	 * @param binWidth width of the histogram bins, in ns
	 * @param topValuesCount number of top values to store
	 */
	explicit TimeMeasure(int histogramBins, uint64_t binWidth = 1000, int topValuesCount = 10);

	void add(uint64_t ns);

	void saveHistCSV(std::string filename);

	void saveTopNCSV(std::string filename);

private:
	std::vector<unsigned int> histogram;
	int histogramBins;
	// ns
	uint64_t histogramBinWidth;
	int histogramBinMax;
	int histogramOverflows;

	int index;
	int topValuesCount;
	std::vector<uint64_t> topValues;
	std::vector<int> topValuesIndices;
};
