/*

This file is a re-implementation of generate_tiles.py from mapnik-stylesheets:
https://github.com/openstreetmap/mapnik-stylesheets/blob/master/generate_tiles.py

*/

#include <mapnik/map.hpp>
#include <mapnik/datasource.hpp>
#include <mapnik/datasource_cache.hpp>
#include <mapnik/load_map.hpp>
#include <mapnik/agg_renderer.hpp>
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

const double DEG_TO_RAD = M_PI / 180;
const double RAD_TO_DEG = 180 / M_PI;

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
    const boost::filesystem::path tileUri;
    const int x;
    const int y;
    const unsigned int z;
};

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

            std::cout << task.tileUri.string() << std::endl;
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
    RenderThread(const boost::filesystem::path &tileDir, const boost::filesystem::path &mapfile, Queue &queue, const unsigned int maxZoom)
        : tileDir(tileDir),
          m(256, 256),
          queue(queue),
          tileproj(maxZoom + 1)
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
        const mapnik::geometry::point<int> p0 = {task.x * 256, (task.y + 1) * 256};
        const mapnik::geometry::point<int> p1 = {(task.x + 1) * 256, (task.y) * 256};

        const mapnik::geometry::point<double> l0 = tileproj.fromPixeltoLL(p0, task.z);
        const mapnik::geometry::point<double> l1 = tileproj.fromPixeltoLL(p1, task.z);

        mapnik::geometry::point<double> c0 = l0;
        mapnik::geometry::point<double> c1 = l1;
        prj->forward(c0.x, c0.y);
        prj->forward(c1.x, c1.y);

        const mapnik::box2d<double> bbox(c0.x, c0.y, c1.x, c1.y);

        int renderSize = 256;
        m.resize(renderSize, renderSize);
        m.zoom_to_box(bbox);
        if (m.buffer_size() < 128)
        {
            m.set_buffer_size(128);
        }

        mapnik::image_rgba8 im(renderSize, renderSize);
        mapnik::agg_renderer<mapnik::image_rgba8> ren(m, im);
        ren.apply();
        mapnik::save_to_file(im, task.tileUri.string());
    }

private:
    const boost::filesystem::path &tileDir;
    mapnik::Map m;
    Queue &queue;
    GoogleProjection tileproj;

    std::unique_ptr<mapnik::projection> prj;
    std::thread thread;
};

void renderTiles(const mapnik::box2d<double> bbox, const boost::filesystem::path &mapfile, const boost::filesystem::path &tileDir, const unsigned int minZoom, const unsigned int maxZoom, const unsigned int numThreads, const bool tmsScheme)
{
    Queue queue;

    std::vector<std::unique_ptr<RenderThread>> renderers;
    renderers.reserve(numThreads);

    for (int i = 0; i < numThreads; ++i)
    {
        renderers.emplace_back(std::make_unique<RenderThread>(tileDir, mapfile, queue, maxZoom));
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

        const int x_first = std::max(0, (int)floor(px0.x / 256.0));
        const int x_last = std::min((int)pow(2, z), (int)floor(px1.x / 256.0));
        for (int x = x_first; x <= x_last; ++x)
        {
            const std::string name_x = std::to_string(x);

            const boost::filesystem::path dir_x = dir_z / name_x;
            if (!boost::filesystem::is_directory(dir_x))
            {
                boost::filesystem::create_directory(dir_x);
            }

            const int y_first = std::max(0, (int)floor(px0.y / 256.0));
            const int y_last = std::min((int)pow(2, z), (int)floor(px1.y / 256.0));

            for (int y = y_first; y <= y_last; ++y)
            {
                const std::string name_y = tmsScheme ? std::to_string(pow(2, z - 1) - y) : std::to_string(y);

                const boost::filesystem::path tileFile = dir_x / (name_y + ".png");
                if (!boost::filesystem::is_regular_file(tileFile))
                {
                    queue.add({tileFile, x, y, z});
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
        renderTiles(bbox, xml, output, zoom_min, zoom_max, threads, false);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}