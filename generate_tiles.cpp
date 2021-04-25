/*

This file is a re-implementation of generate_tiles.py from mapnik-stylesheets:
https://github.com/openstreetmap/mapnik-stylesheets/blob/master/generate_tiles.py

*/

#include <mapnik/map.hpp>
#include <mapnik/datasource.hpp>
#include <mapnik/datasource_cache.hpp>
#include <mapnik/load_map.hpp>
#include <mapnik/agg_renderer.hpp>
#include <mapnik/cairo/cairo_renderer.hpp>
#include <cairo/cairo-svg.h>
#include <mapnik/image.hpp>
#include <mapnik/image_util.hpp>
#include <mapnik/projection.hpp>

#include <cmath>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string/replace.hpp>

const double DEG_TO_RAD = M_PI / 180;
const double RAD_TO_DEG = 180 / M_PI;

const std::string extensionPng = ".png";
const std::string extensionSvg = ".svg";

double minmax(const double a, const double b, const double c)
{
    double result = a;
    result = std::max(result, b);
    result = std::min(result, c);
    return result;
}

class GoogleProjection
{
public:
    GoogleProjection(const int levels)
    {
        double c = 256;
        for (int d = 0; d < levels; ++d)
        {
            double e = c / 2;
            Bc.push_back(c / 360.0);
            Cc.push_back(c / (2 * M_PI));
            zc.push_back(e);
            Ac.push_back(c);
            c *= 2;
        }
    };

    mapnik::geometry::point<int> fromLLtoPixel(const mapnik::geometry::point<double> ll, const unsigned int zoom)
    {
        double d = zc[zoom];
        int e = round(d + ll.x * Bc[zoom]);
        double f = minmax(sin(DEG_TO_RAD * ll.y), -0.9999, 0.9999);
        int g = round(d + 0.5 * log((1 + f) / (1 - f)) * -Cc[zoom]);
        return {e, g};
    }

    mapnik::geometry::point<double> fromPixeltoLL(const mapnik::geometry::point<int> px, const unsigned int zoom)
    {
        double e = zc[zoom];
        double f = (px.x - e) / Bc[zoom];
        double g = (px.y - e) / -Cc[zoom];
        double h = RAD_TO_DEG * (2 * atan(exp(g)) - 0.5 * M_PI);
        return {f, h};
    }

private:
    std::vector<double> Bc;
    std::vector<double> Cc;
    std::vector<double> zc;
    std::vector<double> Ac;
};

struct Task
{
    const boost::filesystem::path tilePathWithoutExtension;
    const int x;
    const int y;
    const unsigned int z;
};

struct CommonTaskSettings
{
    double scale;
    bool png;
    bool svg;
    std::string convertPng;
    std::string convertSvg;
    std::string extensionConvertedPng;
    std::string extensionConvertedSvg;
    bool deletePng;
    bool deleteSvg;
    bool leaveSmallest;
    int tileSize;
    int bufferSize;
};

std::vector<std::string> getPossibleExtensions(const CommonTaskSettings &commonTaskSettings)
{
    std::vector<std::string> possibleExtensions;

    if (commonTaskSettings.png)
    {
        if (commonTaskSettings.convertPng == "" || !commonTaskSettings.deletePng)
        {
            possibleExtensions.push_back(extensionPng);
        }

        if (commonTaskSettings.convertPng != "")
        {
            possibleExtensions.push_back(commonTaskSettings.extensionConvertedPng);
        }
    }

    if (commonTaskSettings.svg)
    {
        if (commonTaskSettings.convertSvg == "" || !commonTaskSettings.deleteSvg)
        {
            possibleExtensions.push_back(extensionSvg);
        }

        if (commonTaskSettings.convertSvg != "")
        {
            possibleExtensions.push_back(commonTaskSettings.extensionConvertedSvg);
        }
    }

    return possibleExtensions;
}

class Queue
{

public:
    Queue() {}

    Queue(const Queue &orig) = delete;
    Queue(Queue &&orig) = delete;
    Queue &operator=(const Queue &orig) = delete;
    Queue &operator=(const Queue &&orig) = delete;

public:
    void add(Task task)
    {
        std::unique_lock<std::mutex> lock(mutex);

        while (queue.size() > 100)
        {
            cv_fromConsumer.wait(lock);
        }

        queue.push(std::move(task));
        cv_fromProducer.notify_one();
    }

    void reportFinish()
    {
        std::lock_guard<std::mutex> lock(mutex);
        finished = true;
        cv_fromProducer.notify_all();
    }

    boost::optional<Task> get()
    {
        std::unique_lock<std::mutex> lock(mutex);
        while (queue.empty() && !finished)
        {
            cv_fromProducer.wait(lock);
        }

        if (queue.empty())
        {
            return boost::none;
        }
        else
        {
            Task task = queue.front();
            queue.pop();
            cv_fromConsumer.notify_one();

            std::cout << task.tilePathWithoutExtension.string() << std::endl;
            return task;
        }
    }

private:
    std::queue<Task> queue;
    bool finished = false;

    std::mutex mutex;
    std::condition_variable cv_fromProducer;
    std::condition_variable cv_fromConsumer;
};

class RenderThread
{
public:
    RenderThread(const boost::filesystem::path &tileDir, const boost::filesystem::path &mapfile, Queue &queue, const unsigned int maxZoom, const CommonTaskSettings &commonTaskSettings)
        : tileDir(tileDir),
          m(commonTaskSettings.tileSize, commonTaskSettings.tileSize),
          queue(queue),
          tileproj(maxZoom + 1),
          commonTaskSettings(commonTaskSettings)
    {
        mapnik::load_map(m, mapfile.string(), true);

        prj = std::make_unique<mapnik::projection>(m.srs());

        thread = std::thread(&RenderThread::threadFunc, this);
    }

    RenderThread(const RenderThread &orig) = delete;
    RenderThread(RenderThread &&orig) = delete;
    RenderThread &operator=(const RenderThread &orig) = delete;
    RenderThread &operator=(const RenderThread &&orig) = delete;

    ~RenderThread()
    {
        thread.join();
    }

private:
    void threadFunc()
    {
        while (true)
        {
            boost::optional<Task> task = queue.get();

            if (!task)
            {
                return;
            }

            renderTile(*task);
        }
    }

private:
    void renderTile(const Task &task)
    {
        const mapnik::geometry::point<int> p0 = {task.x * commonTaskSettings.tileSize, (task.y + 1) * commonTaskSettings.tileSize};
        const mapnik::geometry::point<int> p1 = {(task.x + 1) * commonTaskSettings.tileSize, (task.y) * commonTaskSettings.tileSize};

        const mapnik::geometry::point<double> l0 = tileproj.fromPixeltoLL(p0, task.z);
        const mapnik::geometry::point<double> l1 = tileproj.fromPixeltoLL(p1, task.z);

        mapnik::geometry::point<double> c0 = l0;
        mapnik::geometry::point<double> c1 = l1;
        prj->forward(c0.x, c0.y);
        prj->forward(c1.x, c1.y);

        const mapnik::box2d<double> bbox(c0.x, c0.y, c1.x, c1.y);

        int renderSize = commonTaskSettings.tileSize;
        m.resize(renderSize, renderSize);
        m.zoom_to_box(bbox);
        m.set_buffer_size(commonTaskSettings.bufferSize);

        if (commonTaskSettings.png)
        {
            std::string pngPath = task.tilePathWithoutExtension.string() + extensionPng;

            mapnik::image_rgba8 im(renderSize, renderSize);
            mapnik::agg_renderer<mapnik::image_rgba8> ren(m, im, commonTaskSettings.scale);
            ren.apply();
            mapnik::save_to_file(im, pngPath);

            if (commonTaskSettings.convertPng != "")
            {
                std::string pngPathConverted = task.tilePathWithoutExtension.string() + commonTaskSettings.extensionConvertedPng;

                std::string commandLine = boost::replace_all_copy(commonTaskSettings.convertPng, "$1", pngPath);
                commandLine = boost::replace_all_copy(commandLine, "$2", pngPathConverted);

                {
                    int result = ::system(commandLine.c_str());

                    if (result != 0)
                    {
                        throw "Invalid conversion result: " + std::to_string(result);
                    }
                }

                if (commonTaskSettings.deletePng)
                {
                    boost::filesystem::remove(pngPath);
                }
            }
        }

        if (commonTaskSettings.svg)
        {
            std::string svgPath = task.tilePathWithoutExtension.string() + extensionSvg;

            cairo_surface_t *surface = cairo_svg_surface_create(svgPath.c_str(), renderSize, renderSize);
            mapnik::cairo_surface_ptr surfacePtr(cairo_surface_reference(surface), mapnik::cairo_surface_closer());
            mapnik::cairo_renderer<mapnik::cairo_ptr> ren(m, mapnik::create_context(surfacePtr), commonTaskSettings.scale);
            ren.apply();
            cairo_surface_finish(surface);

            if (commonTaskSettings.convertSvg != "")
            {
                std::string svgPathConverted = task.tilePathWithoutExtension.string() + commonTaskSettings.extensionConvertedSvg;

                std::string commandLine = boost::replace_all_copy(commonTaskSettings.convertSvg, "$1", svgPath);
                commandLine = boost::replace_all_copy(commandLine, "$2", svgPathConverted);

                {
                    int result = ::system(commandLine.c_str());

                    if (result != 0)
                    {
                        throw "Invalid conversion result: " + std::to_string(result);
                    }
                }

                if (commonTaskSettings.deleteSvg)
                {
                    boost::filesystem::remove(svgPath);
                }
            }
        }

        if (commonTaskSettings.leaveSmallest)
        {
            const std::vector<std::string> possibleExtensions = getPossibleExtensions(commonTaskSettings);

            const std::string extensionWithMinSize = *std::min_element(possibleExtensions.begin(), possibleExtensions.end(), [&task](const std::string &extensionA, const std::string &extensionB) {
                return boost::filesystem::file_size(task.tilePathWithoutExtension.string() + extensionA) < boost::filesystem::file_size(task.tilePathWithoutExtension.string() + extensionB);
            });

            for (const std::string &extension : possibleExtensions)
            {
                if (extension != extensionWithMinSize)
                {
                    boost::filesystem::remove(task.tilePathWithoutExtension.string() + extension);
                }
            }
        }
    }

private:
    const boost::filesystem::path &tileDir;
    mapnik::Map m;
    Queue &queue;
    GoogleProjection tileproj;

    std::unique_ptr<mapnik::projection> prj;
    std::thread thread;

    const CommonTaskSettings &commonTaskSettings;
};

bool alreadyCreated(const CommonTaskSettings &commonTaskSettings, const boost::filesystem::path &tilePathWithoutExtension)
{
    const std::vector<std::string> possibleExtensions = getPossibleExtensions(commonTaskSettings);

    if (commonTaskSettings.leaveSmallest)
    {
        return std::any_of(possibleExtensions.begin(), possibleExtensions.end(), [&tilePathWithoutExtension](const std::string &extension) { return boost::filesystem::is_regular_file(tilePathWithoutExtension.string() + extension); });
    }
    else
    {
        return std::all_of(possibleExtensions.begin(), possibleExtensions.end(), [&tilePathWithoutExtension](const std::string &extension) { return boost::filesystem::is_regular_file(tilePathWithoutExtension.string() + extension); });
    }
}

void renderTiles(const mapnik::box2d<double> bbox, const boost::filesystem::path &mapfile, const boost::filesystem::path &tileDir, const unsigned int minZoom, const unsigned int maxZoom, const CommonTaskSettings &commonTaskSettings, const unsigned int numThreads, const bool tmsScheme)
{
    Queue queue;

    std::vector<std::unique_ptr<RenderThread>> renderers;
    renderers.reserve(numThreads);

    for (int i = 0; i < numThreads; ++i)
    {
        renderers.emplace_back(std::make_unique<RenderThread>(tileDir, mapfile, queue, maxZoom, commonTaskSettings));
    }

    if (!boost::filesystem::is_directory(tileDir))
    {
        boost::filesystem::create_directories(tileDir);
    }

    GoogleProjection gprj(maxZoom + 1);

    const mapnik::geometry::point<double> ll0 = {bbox[0], bbox[3]};
    const mapnik::geometry::point<double> ll1 = {bbox[2], bbox[1]};

    for (unsigned int z = minZoom; z <= maxZoom; ++z)
    {
        const std::string name_z = std::to_string(z);
        const boost::filesystem::path dir_z = tileDir / name_z;
        if (!boost::filesystem::is_directory(dir_z))
        {
            boost::filesystem::create_directory(dir_z);
        }

        mapnik::geometry::point<int> px0 = gprj.fromLLtoPixel(ll0, z);
        mapnik::geometry::point<int> px1 = gprj.fromLLtoPixel(ll1, z);

        const int x_first = std::max(0, (int)floor(px0.x / (double)commonTaskSettings.tileSize));
        const int x_last = std::min((int)pow(2, z) * 256 / commonTaskSettings.tileSize - 1, (int)floor(px1.x / (double)commonTaskSettings.tileSize));
        for (int x = x_first; x <= x_last; ++x)
        {
            const std::string name_x = std::to_string(x);

            const boost::filesystem::path dir_x = dir_z / name_x;
            if (!boost::filesystem::is_directory(dir_x))
            {
                boost::filesystem::create_directory(dir_x);
            }

            const int y_first = std::max(0, (int)floor(px0.y / (double)commonTaskSettings.tileSize));
            const int y_last = std::min((int)pow(2, z) * 256 / commonTaskSettings.tileSize - 1, (int)floor(px1.y / (double)commonTaskSettings.tileSize));

            for (int y = y_first; y <= y_last; ++y)
            {
                const std::string name_y = tmsScheme ? std::to_string(pow(2, z - 1) - y) : std::to_string(y);

                const boost::filesystem::path tilePathWithoutExtension = dir_x / name_y;
                if (!alreadyCreated(commonTaskSettings, tilePathWithoutExtension))
                {
                    queue.add({tilePathWithoutExtension, x, y, z});
                }
            }
        }
    }

    queue.reportFinish();
}

void initMapnik(const boost::filesystem::path &datasources, const boost::filesystem::path &fonts)
{
    mapnik::datasource_cache::instance().register_datasources(datasources.string());

    mapnik::freetype_engine::register_fonts(fonts.string(), true);

    std::cout << "Available fonts:" << std::endl;
    for (auto const &fn : mapnik::freetype_engine::face_names())
    {
        std::cout << fn << std::endl;
    }
}

int main(int argc, char *argv[])
{
    try
    {
        boost::filesystem::path xml;
        boost::filesystem::path output;
        double lat_min;
        double lat_max;
        double lon_min;
        double lon_max;
        unsigned int zoom_min;
        unsigned int zoom_max;
        unsigned int threads;
        bool tms;

        boost::filesystem::path datasources;
        boost::filesystem::path fonts;

        boost::program_options::options_description desc("Allowed options");

        desc.add_options()("help,h", "print usage message");

        desc.add_options()("xml", boost::program_options::value(&xml)->required(), "map XML file");
        desc.add_options()("output", boost::program_options::value(&output)->required(), "output directory for tiles");

        desc.add_options()("lat_min", boost::program_options::value(&lat_min)->default_value(-90.0), "process area from latitude");
        desc.add_options()("lat_max", boost::program_options::value(&lat_max)->default_value(90.0), "process area to latitude");
        desc.add_options()("lon_min", boost::program_options::value(&lon_min)->default_value(-180.0), "process area from longitude");
        desc.add_options()("lon_max", boost::program_options::value(&lon_max)->default_value(180.0), "process area to longitude");
        desc.add_options()("zoom_min", boost::program_options::value(&zoom_min)->default_value(0), "min. zoom to render tiles for");
        desc.add_options()("zoom_max", boost::program_options::value(&zoom_max)->default_value(5), "max. zoom to render tiles for");

        desc.add_options()("threads", boost::program_options::value(&threads)->default_value(1), "count of threads to use for rendering");
        desc.add_options()("tms", boost::program_options::value(&tms)->default_value(false), "use tms scheme (flip y)");

        desc.add_options()("datasources", boost::program_options::value(&datasources)->default_value("/usr/lib/mapnik/input"), "mapnik data sources directory");
        desc.add_options()("fonts", boost::program_options::value(&fonts)->default_value("/usr/share/fonts"), "fonts directory");

        CommonTaskSettings commonTaskSettings;
        desc.add_options()("scale", boost::program_options::value(&commonTaskSettings.scale)->default_value(1), "scale factor (use 2 for the 2x tiles for high-resolution displays");
        desc.add_options()("png", boost::program_options::value(&commonTaskSettings.png)->default_value(true), "create PNG (raster) tiles");
        desc.add_options()("svg", boost::program_options::value(&commonTaskSettings.svg)->default_value(false), "create SVG (vector) tiles - note that SVG files for complicated tiles can be extremely large without any converting");
        desc.add_options()("convert_png", boost::program_options::value(&commonTaskSettings.convertPng)->default_value(""), "conversion command to run for each PNG tile - use placeholders for file paths: $1 - absolute path of the original file, $2 - absolute path of the converted file");
        desc.add_options()("convert_svg", boost::program_options::value(&commonTaskSettings.convertSvg)->default_value(""), "conversion command to run for each SVG tile - use placeholders for file paths: $1 - absolute path of the original file, $2 - absolute path of the converted file");
        desc.add_options()("extension_converted_png", boost::program_options::value(&commonTaskSettings.extensionConvertedPng)->default_value(".pngc"), "extension of the converted PNG file");
        desc.add_options()("extension_converted_svg", boost::program_options::value(&commonTaskSettings.extensionConvertedSvg)->default_value(".svgc"), "extension of the converted SVG file");
        desc.add_options()("delete_png", boost::program_options::value(&commonTaskSettings.deletePng)->default_value(false), "delete original PNG file after conversion");
        desc.add_options()("delete_svg", boost::program_options::value(&commonTaskSettings.deleteSvg)->default_value(false), "delete original SVG file after conversion");
        desc.add_options()("leave_smallest", boost::program_options::value(&commonTaskSettings.leaveSmallest)->default_value(false), "leave smallest file amongst all generated files");
        desc.add_options()("tile_size", boost::program_options::value(&commonTaskSettings.tileSize)->default_value(256), "tile size");
        desc.add_options()("buffer_size", boost::program_options::value(&commonTaskSettings.bufferSize)->default_value(128), "Mapnik buffer size - use higher value if some tile boundaries do not match (e.g. if a text is cut at the edge of two tiles)");

        boost::program_options::variables_map vm;
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
            return 0;
        }

        boost::program_options::notify(vm);

        initMapnik(datasources, fonts);

        const mapnik::box2d<double> bbox(lon_min, lat_min, lon_max, lat_max);
        renderTiles(bbox, xml, output, zoom_min, zoom_max, commonTaskSettings, threads, false);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}