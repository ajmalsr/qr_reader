#include "Image.h"
#include "ImageRotation.h"
#include "GrayImage.h"
#include "Clahe.h"
#include "Binarizer.h"
#include "Otsu.h"
#include "HorizontalFinder.h"
#include "FinderValidation.h"
#include "FinderFailureAnalysis.h"
#include "FinderMerging.h"
#include "FinderTripletSelection.h"
#include "ModuleSizeEstimation.h"
#include "QrDimensionEstimation.h"
#include "AlignmentPattern.h"
#include "PerspectiveTransform.h"
#include "GridSampling.h"
#include "NormalizedQrMatrix.h"
#include "FunctionPatternMap.h"
#include "FormatInformation.h"
#include "DataMask.h"
#include "RawCodewordExtraction.h"
#include "ErrorCorrectionBlocks.h"
#include "ReedSolomonCorrection.h"
#include "CorrectedDataStream.h"
#include "QrSegmentParser.h"
#include "FinalDecodedResult.h"
#include "MorphologicalClosing.h"

#include <cctype>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

namespace
{
// Edit this value and rebuild. Positive angles rotate clockwise and negative
// angles rotate anticlockwise. Exactly zero bypasses all rotation code.
constexpr double InputRotationDegrees = 0.0;
}

// Runs the full detection pipeline built up over Milestones 1-5:
//   1. Load PNG -> RGBA (Image)
//   2. RGBA -> luminance (GrayImage)
//   3. Local adaptive binarization (Binarizer) + Otsu comparison (Otsu, diagnostic only)
//   4. Horizontal finder-pattern candidate search (HorizontalFinder): raw 1:1:3:1:1 row hits
//   5. Finder validation (FinderValidation): vertical/horizontal/diagonal cross-checks
// Each stage writes its own debug PNG/TXT output under [selected output directory]/ so failures can be
// traced back to the stage and measurement that caused them.
int main(int argc, char* argv[])
{
    if (argc != 2 && argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <input.png> [--clahe]\n";
        return 1;
    }

    const std::string filename = argv[1];
    // CLAHE is opt-in. Without the flag, the original pipeline and output location
    // behave exactly as before this experiment.
    const bool useClahe = argc == 3 && std::string(argv[2]) == "--clahe";
    if (argc == 3 && !useClahe)
    {
        std::cerr << "Error: unknown option '" << argv[2] << "'. Expected --clahe\n";
        return 1;
    }
    // Keep experimental artifacts isolated so a CLAHE run cannot overwrite the
    // baseline diagnostic images and text reports in output/.
    const std::string outputDirectory = useClahe ? "output/clahe" : "output";
    const auto outputPath = [&](const std::string& name) {
        return (std::filesystem::path(outputDirectory) / name).string();
    };
    std::cout << "Input filename: " << filename << '\n'
              << "CLAHE mode: " << (useClahe ? "enabled" : "disabled") << '\n'
              << "Output directory: " << outputDirectory << '\n';

    try
    {
        std::string extension = std::filesystem::path(filename).extension().string();
        for (char& character : extension)
        {
            character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
        }
        if (extension != ".png")
        {
            std::cerr << "Error: input must have a .png extension\n";
            return 1;
        }

        Image image = loadPng(filename);
        if (InputRotationDegrees != 0.0)
        {
            image = rotateImage(image, InputRotationDegrees);
            std::cout << "Input rotation: " << InputRotationDegrees
                      << " degrees (positive is clockwise)\n";
        }
        else
        {
            std::cout << "Input rotation: disabled (0 degrees)\n";
        }
        std::cout << "Width: " << image.width << '\n'
                  << "Height: " << image.height << '\n'
                  << "Channels: " << Image::channels << '\n'
                  << "Pixel count: " << image.pixelCount() << '\n'
                  << "Total RGBA bytes: " << image.rgba.size() << '\n';

        // Always retain and save the unmodified luminance image. The downstream
        // pipeline receives a copy, optionally replaced by CLAHE output below.
        const GrayImage originalGray = toLuminance(image);
        std::filesystem::create_directories(outputDirectory);
        const std::string outputFilename = outputPath("01_luminance.png");
        saveGrayPng(originalGray, outputFilename);
        GrayImage gray = originalGray;
        // Only this branch differs from normal mode. CLAHE produces a new GrayImage;
        // all existing binarization and finder algorithms remain unchanged.
        if (useClahe)
        {
            const ClaheSettings settings;
            gray = applyClahe(originalGray, settings);
            saveGrayPng(gray, outputPath("01_clahe_luminance.png"));
            saveClaheStatistics(originalGray, gray, settings,
                                outputPath("01_clahe_statistics.txt"));
        }
        std::cout << "Grayscale channels: " << GrayImage::channels << '\n'
                  << "Total luminance bytes: " << gray.pixels.size() << '\n'
                  << "Saved luminance PNG: " << outputFilename << '\n';

        const BinarizationResult binary = binarizeLocal(gray);
        saveGrayPng(binaryForPng(binary.matrix), outputPath("02_binary.png"));
        saveThresholdStatistics(binary, outputPath("02_threshold_statistics.txt"));
        std::cout << "Block grid: " << binary.blocksWide << " x " << binary.blocksHigh << '\n'
                  << "Saved binary PNG: [selected output directory]/02_binary.png\n"
                  << "Saved threshold statistics: [selected output directory]/02_threshold_statistics.txt\n";

        const OtsuResult otsu = binarizeOtsu(gray);
        saveGrayPng(binaryForPng(otsu.matrix), outputPath("03_otsu.png"));
        const GrayImage difference = saveBinarizationComparison(
            binary, otsu, outputPath("03_binarization_comparison.txt"));
        saveGrayPng(difference, outputPath("04_binarization_difference.png"));
        std::cout << "Otsu threshold: " << otsu.threshold << '\n'
                  << "Otsu valid split: " << otsu.hasValidSplit << '\n'
                  << "Saved Otsu PNG: [selected output directory]/03_otsu.png\n"
                  << "Saved comparison: [selected output directory]/03_binarization_comparison.txt\n"
                  << "Saved difference PNG: [selected output directory]/04_binarization_difference.png\n";

        // Experimental, optional pre-processing step (not part of Milestones 1-5): 3x3
        // morphological closing on the binary matrix, meant to repair small finder-ring
        // fragmentation before finder search runs. Kept in its own translation unit
        // (MorphologicalClosing.h/.cpp); removing this block and that file pair fully
        // reverts finder search/validation to running on the original binary matrix.
        const BitMatrix closed = closeBitMatrix3x3(binary.matrix);
        saveGrayPng(binaryForPng(closed), outputPath("05_morphological_closing.png"));
        saveMorphologicalClosingDebug(binary.matrix, closed, outputPath("05_morphological_closing_stats.txt"));
        std::cout << "Saved morphologically closed binary PNG: [selected output directory]/05_morphological_closing.png\n"
                  << "Saved morphological closing statistics: [selected output directory]/05_morphological_closing_stats.txt\n";

        const HorizontalFinderResult horizontal = searchHorizontalFinders(closed);
        saveHorizontalFinderDebug(image, horizontal,
            outputPath("03_horizontal_finder_hits.png"), outputPath("03_horizontal_finder_hits.txt"));
        std::cout << "Rows scanned: " << horizontal.rowsScanned << '\n'
                  << "5-run windows tested (black-starting): " << horizontal.windowsTested << '\n'
                  << "Fast-plausibility passes: " << horizontal.fastPasses << '\n'
                  << "Final ratio passes: " << horizontal.ratioPasses << '\n'
                  << "Horizontal hits: " << horizontal.hits.size() << '\n'
                  << "Saved horizontal hit PNG and TXT in [selected output directory]/\n";

        const FinderValidationResult validation = validateFinderHits(closed, horizontal.hits);
        saveFinderValidationDebug(image, validation, outputDirectory);
        std::cout << "Raw horizontal hits: " << validation.records.size() << '\n'
                  << "Vertical scans attempted: " << validation.verticalScansAttempted << '\n'
                  << "Incomplete vertical scans: " << validation.incompleteVerticalScans << '\n'
                  << "Vertical ratio failures: " << validation.verticalRatioFailures << '\n'
                  << "Size-consistency failures: " << validation.sizeConsistencyFailures << '\n'
                  << "Center-consistency failures: " << validation.centerConsistencyFailures << '\n'
                  << "Horizontal re-check failures: " << validation.horizontalRecheckFailures << '\n'
                  << "Diagonal failures (diagnostic): " << validation.diagonalFailures << '\n'
                  << "Vertical passes: " << validation.verticalPasses << '\n'
                  << "Axis passes: " << validation.axisPasses << '\n'
                  << "Diagonal passes: " << validation.diagonalPasses << '\n'
                  << "Final validated candidates (unmerged): " << validation.candidates.size() << '\n';

        // Milestone 7: failure analysis only - no detection/validation logic changes. Reports
        // and visualizes, for every raw Milestone-4 hit, exactly which pipeline stage it
        // reached and why it was accepted or rejected there.
        saveFinderFailureAnalysisText(validation, outputPath("06_finder_failure_analysis.txt"));
        saveHorizontalCandidatesMap(image, validation, outputPath("06_horizontal_candidates.png"));
        saveVerticalFailuresMap(image, validation, outputPath("06_vertical_failures.png"));
        saveFailureReasonsMap(image, validation, outputPath("06_failure_reasons.png"));
        saveAllCandidatesRejectionMap(image, validation, outputPath("06_all_finder_candidates_rejection_map.png"));
        std::cout << "Saved finder failure analysis: [selected output directory]/06_finder_failure_analysis.txt\n"
                  << "Saved horizontal candidates map: [selected output directory]/06_horizontal_candidates.png\n"
                  << "Saved vertical failures map: [selected output directory]/06_vertical_failures.png\n"
                  << "Saved failure reasons map: [selected output directory]/06_failure_reasons.png\n"
                  << "Saved master rejection map: [selected output directory]/06_all_finder_candidates_rejection_map.png\n";

        // Milestone 6: merge repeated validated observations of the same physical finder
        // (typically produced by several adjacent scanned rows) into one cluster per finder.
        const FinderMergingResult merging = mergeFinderObservations(validation.candidates);
        saveFinderMergingDebug(image, validation.candidates, merging, outputDirectory);
        std::cout << "Physical finder clusters: " << merging.clusters.size() << '\n'
                  << "Saved merged finder PNG: [selected output directory]/05_merged_finders.png\n"
                  << "Saved finder cluster statistics: [selected output directory]/05_finder_clusters.txt\n";

        // Detection stage 9: test every three-cluster combination, explain each
        // rejection, and retain the best geometrically plausible TL/TR/BL triplet.
        // This deliberately stops before module-size and QR-dimension estimation.
        const FinderTripletSelectionResult triplet = selectFinderTriplet(merging.clusters);
        saveFinderTripletSelectionDebug(image, merging.clusters, triplet, outputDirectory);
        std::size_t validTriplets = 0;
        for (const auto& evaluation : triplet.evaluations)
            if (evaluation.accepted) ++validTriplets;
        std::cout << "Triplet combinations tested: " << triplet.evaluations.size() << '\n'
                  << "Valid finder triplets: " << validTriplets << '\n'
                  << "Rejected finder triplets: " << triplet.evaluations.size() - validTriplets << '\n'
                  << "Finder triplet selected: " << (triplet.hasSelection ? "yes" : "no") << '\n';
        if (triplet.hasSelection)
            std::cout << "Selected finder IDs (TL,TR,BL): "
                      << triplet.selectedCandidateIndices[0] << ','
                      << triplet.selectedCandidateIndices[1] << ','
                      << triplet.selectedCandidateIndices[2] << '\n';
        else
            std::cout << "Triplet selection failure: " << triplet.overallFailureReason << '\n';
        std::cout << "Saved triplet selection PNG and TXT in [selected output directory]/\n";

        // Detection stage 10: combine the three selected finder measurements into one
        // validated pixels-per-module estimate. Dimension estimation remains Stage 11.
        const ModuleSizeEstimationResult moduleSize =
            estimateSelectedTripletModuleSize(merging.clusters, triplet);
        saveModuleSizeEstimationReport(moduleSize,
            outputPath("08_module_size_estimation.txt"), filename);
        std::cout << "Module-size estimation: " << (moduleSize.valid ? "accepted" : "rejected") << '\n';
        if (moduleSize.valid)
            std::cout << "Estimated module size: " << moduleSize.estimatedModuleSize
                      << " pixels/module\n";
        else
            std::cout << "Module-size failure: " << moduleSize.failureReason << '\n';
        std::cout << "Saved module-size report: [selected output directory]/08_module_size_estimation.txt\n";

        // Detection stage 11: estimate a legal Model 2 dimension from the top and
        // left finder-center separations. Alignment-pattern work remains Stage 12.
        const QrDimensionEstimationResult dimension =
            estimateQrDimension(merging.clusters, moduleSize);
        saveQrDimensionEstimationReport(dimension,
            outputPath("09_qr_dimension_estimation.txt"), filename);
        std::cout << "QR dimension estimation: " << (dimension.valid ? "accepted" : "rejected") << '\n';
        if (dimension.valid)
            std::cout << "Estimated QR dimension: " << dimension.dimension << " x "
                      << dimension.dimension << " modules (version " << dimension.version << ")\n";
        else
            std::cout << "QR dimension failure: " << dimension.failureReason << '\n';
        std::cout << "Saved QR dimension report: [selected output directory]/09_qr_dimension_estimation.txt\n";

        // Detection stage 12: predict and cross-check the bottom-right alignment
        // pattern for version 2+, or explicitly skip it for version 1.
        const AlignmentPatternSearchResult alignment =
            searchAlignmentPattern(closed, merging.clusters, moduleSize, dimension);
        saveAlignmentPatternDebug(image, alignment, outputDirectory, filename);
        if (alignment.valid && !alignment.required)
            std::cout << "Alignment pattern: not required for version 1\n";
        else if (alignment.valid)
            std::cout << "Alignment pattern found at: " << alignment.selected.centerX
                      << ',' << alignment.selected.centerY << '\n';
        else
            std::cout << "Alignment pattern failure: " << alignment.failureReason << '\n';
        std::cout << "Saved alignment-pattern PNG and TXT in [selected output directory]/\n";

        // Detection stage 13: construct both directions of the projective mapping.
        // This stage only models geometry; module sampling remains Stage 14.
        const QrPerspectiveTransformResult perspective =
            buildQrPerspectiveTransform(merging.clusters, dimension, alignment);
        savePerspectiveTransformDebug(image, perspective, outputDirectory, filename);
        if (perspective.valid)
            std::cout << "Perspective transform: valid; maximum control reprojection error: "
                      << perspective.maximumForwardReprojectionError << " pixels\n";
        else
            std::cout << "Perspective transform failure: " << perspective.failureReason << '\n';
        std::cout << "Saved perspective-transform PNG and TXT in [selected output directory]/\n";

        // Detection stage 14: project every logical module center and read one
        // source bit. Stage 15 will package successful samples as the normalized matrix.
        const GridSamplingResult gridSampling = sampleQrGrid(closed, perspective);
        saveGridSamplingDebug(image, gridSampling, outputDirectory, filename);
        if (gridSampling.valid)
            std::cout << "Grid sampling: " << gridSampling.samplesRead << " modules read ("
                      << gridSampling.blackSamples << " black, "
                      << gridSampling.whiteSamples << " white)\n";
        else
            std::cout << "Grid sampling failure: " << gridSampling.failureReason << '\n';
        std::cout << "Saved grid-sampling debug outputs in [selected output directory]/\n";
        // Detection stage 15: validate that every logical coordinate has exactly one
        // successful Stage-14 record, then package those bits as a row-major matrix.
        const NormalizedQrMatrixResult normalized = buildNormalizedQrMatrix(gridSampling);
        saveNormalizedQrMatrix(normalized, outputDirectory, filename);
        if (normalized.valid)
            std::cout << "Normalized QR BitMatrix: " << normalized.dimension << " x "
                      << normalized.dimension << " modules ("
                      << normalized.blackModules << " black, "
                      << normalized.whiteModules << " white)\n";
        else
            std::cout << "Normalized QR BitMatrix failure: "
                      << normalized.failureReason << '\n';
        std::cout << (normalized.valid
            ? "Saved normalized QR matrix PNG and TXT in [selected output directory]/\n"
            : "Saved normalized QR matrix rejection TXT in [selected output directory]/\n");

        // Decoding stage 16: reserve every fixed/function module so later mask
        // removal and codeword traversal operate only on data/remainder modules.
        const FunctionPatternMapResult functionMap = buildFunctionPatternMap(normalized);
        saveFunctionPatternMap(functionMap, outputDirectory, filename);
        if (functionMap.valid)
            std::cout << "Function-pattern map: " << functionMap.functionModuleCount
                      << " function modules, "
                      << functionMap.dataOrRemainderModuleCount
                      << " data/remainder modules\n";
        else
            std::cout << "Function-pattern map failure: "
                      << functionMap.failureReason << '\n';
        std::cout << (functionMap.valid
            ? "Saved function-pattern map PNG and TXT in [selected output directory]/\n"
            : "Saved function-pattern map rejection TXT in [selected output directory]/\n");

        // Decoding stage 17: read both BCH-protected format copies to recover
        // the error-correction level and the data-mask number. Do not unmask yet.
        const FormatInformationResult formatInformation =
            decodeFormatInformation(normalized, functionMap);
        saveFormatInformationReport(formatInformation, outputDirectory, filename);
        if (formatInformation.valid)
            std::cout << "Format information: EC level "
                      << errorCorrectionLevelName(formatInformation.errorCorrectionLevel)
                      << ", data mask " << formatInformation.dataMask << '\n';
        else
            std::cout << "Format-information failure: "
                      << formatInformation.failureReason << '\n';
        std::cout << "Saved format-information TXT in [selected output directory]/\n";

        // Decoding stage 18: remove the decoded mask only from data/remainder
        // modules. Function modules and the Stage-15 source matrix stay unchanged.
        const DataMaskRemovalResult dataMaskRemoval =
            removeQrDataMask(normalized, functionMap, formatInformation);
        saveDataMaskRemovalDebug(dataMaskRemoval, outputDirectory, filename);
        if (dataMaskRemoval.valid)
        {
            std::cout << "Data-mask removal: accepted\n";
            std::cout << "Function modules preserved: "
                      << dataMaskRemoval.functionModulesPreserved << '\n';
            std::cout << "Data/remainder modules visited: "
                      << dataMaskRemoval.dataOrRemainderModulesVisited << '\n';
            std::cout << "Modules flipped: " << dataMaskRemoval.modulesFlipped << '\n';
        }
        else
        {
            std::cout << "Data-mask removal failure: "
                      << dataMaskRemoval.failureReason << '\n';
        }
        std::cout << (dataMaskRemoval.valid
            ? "Saved unmasked QR matrix PNG and TXT in [selected output directory]/\n"
            : "Saved data-mask rejection TXT in [selected output directory]/\n");

        // Decoding stage 19: follow the standard two-column QR zigzag,
        // excluding function modules, then pack complete groups MSB-first.
        const RawCodewordExtractionResult rawCodewords =
            extractRawCodewords(dataMaskRemoval, functionMap);
        saveRawCodewordExtractionReport(rawCodewords, outputDirectory, filename);
        if (rawCodewords.valid)
        {
            std::cout << "Raw codeword extraction: accepted\n";
            std::cout << "Raw codewords: " << rawCodewords.rawCodewords.size()
                      << " (" << rawCodewords.codewordBits << " bits)\n";
            std::cout << "Remainder bits: " << rawCodewords.remainderBits.size()
                      << " (all zero: "
                      << (rawCodewords.remainderBitsAllZero ? "yes" : "no")
                      << ")\n";
        }
        else
        {
            std::cout << "Raw-codeword extraction failure: "
                      << rawCodewords.failureReason << '\n';
        }
        std::cout << "Saved raw-codeword TXT in [selected output directory]/\n";

        // Decoding stage 20: use the version/EC layout to reverse QR's
        // position-by-position interleaving into independent RS blocks.
        const ErrorCorrectionBlockResult errorCorrectionBlocks =
            deinterleaveErrorCorrectionBlocks(rawCodewords);
        saveErrorCorrectionBlockReport(
            errorCorrectionBlocks, outputDirectory, filename);
        if (errorCorrectionBlocks.valid)
        {
            std::cout << "Error-correction block layout: accepted\n";
            std::cout << "Blocks: "
                      << errorCorrectionBlocks.layout.totalBlocks
                      << ", data codewords: "
                      << errorCorrectionBlocks.layout.totalDataCodewords
                      << ", EC codewords: "
                      << errorCorrectionBlocks.layout
                             .totalErrorCorrectionCodewords
                      << '\n';
        }
        else
        {
            std::cout << "Error-correction block failure: "
                      << errorCorrectionBlocks.failureReason << '\n';
        }
        std::cout << "Saved error-correction block TXT in "
                     "[selected output directory]/\n";

        // Decoding stage 21: correct each independent QR block in GF(256).
        // Later stages will concatenate only the corrected data portions.
        const ReedSolomonCorrectionResult reedSolomon =
            correctReedSolomonBlocks(errorCorrectionBlocks);
        saveReedSolomonCorrectionReport(
            reedSolomon, outputDirectory, filename);
        if (reedSolomon.valid)
        {
            std::cout << "Reed-Solomon correction: accepted\n";
            std::cout << "Blocks with errors: "
                      << reedSolomon.blocksWithErrors
                      << ", corrected codewords: "
                      << reedSolomon.totalCorrectedCodewords
                      << '\n';
        }
        else
        {
            std::cout << "Reed-Solomon correction failure: "
                      << reedSolomon.failureReason << '\n';
        }
        std::cout << "Saved Reed-Solomon correction TXT in "
                     "[selected output directory]/\n";

        // Decoding stage 22: restore the logical data-byte sequence by
        // concatenating each corrected block's data portion in block order.
        const CorrectedDataStreamResult correctedDataStream =
            reconstructCorrectedDataStream(
                reedSolomon, errorCorrectionBlocks);
        saveCorrectedDataStreamReport(
            correctedDataStream, outputDirectory, filename);
        if (correctedDataStream.valid)
        {
            std::cout << "Corrected data stream: "
                      << correctedDataStream.bytes.size() << " bytes ("
                      << correctedDataStream.bitCount << " bits)\n";
        }
        else
        {
            std::cout << "Corrected data-stream failure: "
                      << correctedDataStream.failureReason << '\n';
        }
        std::cout << "Saved corrected data-stream TXT in "
                     "[selected output directory]/\n";

        // Decoding stage 23: interpret only the supported QR segment modes.
        // Final payload assembly and presentation remain a separate stage.
        const QrSegmentParsingResult parsedSegments =
            parseQrSegments(correctedDataStream);
        saveQrSegmentParsingReport(
            parsedSegments, outputDirectory, filename);
        if (parsedSegments.valid)
        {
            std::cout << "Parsed QR segments: "
                      << parsedSegments.segments.size() << '\n';
        }
        else
        {
            std::cout << "QR segment parsing failure: "
                      << parsedSegments.failureReason << '\n';
        }
        std::cout << "Saved parsed-segment TXT in "
                     "[selected output directory]/\n";

        // Decoding stage 24: publish one result only after every parsed
        // segment passes consistency checks. Raw byte segments stay attached.
        const FinalDecodedResult finalResult =
            assembleFinalDecodedResult(correctedDataStream, parsedSegments);
        saveFinalDecodedResultReport(finalResult, outputDirectory, filename);
        if (finalResult.valid)
        {
            std::cout << "Decoded QR result: " << finalResult.text << '\n';
        }
        else
        {
            std::cout << "Final decoded-result failure: "
                      << finalResult.failureReason << '\n';
        }
        std::cout << "Saved final decoded-result TXT in "
                     "[selected output directory]/\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}



