#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>
#include <QThread>
#include <map>
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>

#include "upolis_control/matplotlibcpp.h"

namespace plt = matplotlibcpp;

class Worker : public QObject {
    Q_OBJECT
    std::map<std::string, std::vector<std::pair<double, double>>> graphs;
    std::mutex mutex;
    bool running = true;
public:
    void addPoint(const std::string& graphName, double x, double y) {
        std::lock_guard<std::mutex> guard(mutex);
        graphs[graphName].emplace_back(x, y);
    }

    void updatePlot() {
        std::lock_guard<std::mutex> guard(mutex);
        plt::clf(); // Clear current figure
        for (const auto& graph : graphs) {
            std::vector<double> x, y;
            for (const auto& point : graph.second) {
                x.push_back(point.first);
                y.push_back(point.second);
            }
            plt::plot(x, y); // Assuming plot supports this usage
        }
        std::cout << "In loop 2" << std::endl;
        plt::legend();
        plt::pause(0.01); // Short pause to update plot, might need adjustment for Qt
    }

public Q_SLOTS:
    void process() {
        while (running) {
            std::cout << "In loop 1 " << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(15)); // Control refresh rate
            updatePlot();
        }
        Q_EMIT finished("Task completed");
    }

Q_SIGNALS:
    void finished(QString result);
};

class MainWindow : public QWidget {
    Q_OBJECT
public:
    MainWindow() {
        auto *layout = new QVBoxLayout(this);
        auto *button = new QPushButton("Start Background Task", this);
        auto *label = new QLabel("Press the button", this);
        layout->addWidget(button);
        layout->addWidget(label);
        plt::figure();
        auto *worker = new Worker();
        auto *thread = new QThread();
        worker->moveToThread(thread);

        connect(button, &QPushButton::clicked, [thread](){ 
            thread->start(); 
        });
        connect(thread, &QThread::started, worker, &Worker::process);
        connect(thread, &QThread::started, worker, [worker](){
            QMetaObject::invokeMethod(worker, [=](){
                worker->addPoint("Graph 1", rand() % 100, rand() % 100);
            }, Qt::QueuedConnection);
        });
        connect(worker, &Worker::finished, label, [label](QString result){ label->setText(result); });
        connect(worker, &Worker::finished, thread, &QThread::quit);
        connect(thread, &QThread::finished, label, [label](){ QThread::sleep(2); label->setText("Press again"); });
        // connect(thread, &QThread::finished, thread, &QThread::deleteLater);
        plt::show();
    }
};

#include "plotting.moc"

int main(int argc, char *argv[]) {
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}
