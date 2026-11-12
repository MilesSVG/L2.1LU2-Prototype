#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <chrono>
#include <thread>
#include <random>
#include <algorithm>
#include <limits>

// using namespace std;
// using namespace std::chrono;

enum class RobotState
{
    IDLE,
    BUSY,
    FAILED
};

struct Robot
{
    int id;
    RobotState state;

    double expectedTravelTime;
    double actualTravelTime;

    double remainingTime;
    double averageTravelTime;

    double availableAt;
    double recoveryAt;

    int tasksCompleted;

};

enum class FailureReason {
    NONE,
    TIMEOUT,
    NO_REPLACEMENT
};



class CsvLogger
{
private:
    std::ofstream file;
    std::string fileName;

public:

    void SetFileName(const std::string& fileN)
    {
        fileName = fileN + ".csv";
    }
    void StartWriting() {
        file.open(fileName);

        file
            << "taskId,"
            << "assignedRobotId,"
            << "completedRobotId,"
            << "expectedTravelTime,"
            << "actualTravelTime,"
            << "creationTick,"
            << "startTick,"
            << "finishTick,"
            << "waitingTime,"//waiting time starttick - creationtick
            // << "failureReason,"//failureReason
            << "RecoveryTime,"//recoveryTime reallocationtick - failuretick
            << "blockageTime,"
            << "timeout,"
            << "reallocated,"
            << "reallocationTick,"
            << "unnecessaryAbort,"
            << "completed,"
            << "dynamicTimeout,"
            << "dynamicReallocation\n";
    }

    void log(
        int taskId,
        int assignedRobotId,
        int completedRobotId,
        double expectedTravelTime,
        double actualTravelTime,
        double creationTick,
        double startTick,
        double finishTick,
        double waitingTime,
        double recoveryTime,
        double blockageTime,
        bool timeout,
        bool reallocated,
        double reallocationTick,
        bool unnecessaryAbort,
        bool completed,
        bool dynamicTimeout,
        bool dynamicReallocation)
    {
        file
            << taskId << ","
            << assignedRobotId << ","
            << completedRobotId << ","
            << expectedTravelTime << ","
            << actualTravelTime << ","
            << creationTick << ","
            << startTick << ","
            << finishTick << ","
            << waitingTime << ","
            // << failureReason << ","
            << recoveryTime << ","
            << blockageTime << ","
            << timeout << ","
            << reallocated << ","
            << reallocationTick << ","
            << unnecessaryAbort << ","
            << completed << ","
            << dynamicTimeout << ","
            << dynamicReallocation
            << "\n";
    }

    ~CsvLogger()
    {
        file.close();
    }
};

class CsvScenarioLogger
{
private:
    std::ofstream file;
    std::string fileName;

public:

    void SetFileName(const std::string& fileN)
    {
        fileName = fileN + "_Scenario.csv";
    }
    void StartWriting() {
        file.open(fileName);

        file
            << "Tasks,"
            << "Robots,"
            << "FailureRate,"
            << "Completed,"
            << "Timeouts,"
            << "UnnecessaryTimeouts,"
            << "AverageBlockageTime,"
            << "Reallocations,"
            << "Throughput,"
            << "AverageTaskTime"
            << "\n";
    }

    void log(
        int tasks,
        int robots,
        double failureRate,
        int completed,
        int timeouts,
        int unnecessaryTimeouts,
        double averageBlockageTime,
        int reallocations,
        double throughput,
        double averageTaskTime)
    {
        file
            << tasks << ","
            << robots << ","
            << failureRate << ","
            << completed << ","
            << timeouts << ","
            << unnecessaryTimeouts << ","
            << averageBlockageTime << ","
            << reallocations << ","
            << throughput << ","
            << averageTaskTime
            << "\n";
    }

    ~CsvScenarioLogger()
    {
        file.close();
    }
};




class FleetManager
{
private:
    std::vector<Robot> robots;
    std::vector<double> previousTravelTimes;
    CsvLogger logger;
    CsvScenarioLogger runSetting;

    std::mt19937 rng;

    double simulationTime = 0.0;

    int completedTasks = 0;
    int timeoutTasks = 0;

    int unnecessaryAborts = 0;
    double abortMargin = 0.10; // percentage at which the timeout will be considered unnecessary
    // (eg. 0.2 -> 20% so a timeout when actualTravelTime would be less than 20% longer than timeout would be considered unnecessary)

    double averageTaskTime = 0;

    double totalBlockageTime = 0;
    double averageBlockageTime = 0;

    int reallocatedTasks = 0;
    double failureChance = 0.10;

    static constexpr double TASK_ARRIVAL_INTERVAL = 20.0;

    bool dynamicTimeout = false;
    bool dynamicReallocation = false;
    double TIMEOUT_LENGTH = 250;


public:
    FleetManager(const int numberOfRobots, const bool dynTimeout, const bool dynReal, const std::string& fileName)
    {
        logger.SetFileName(fileName);
        logger.StartWriting();

        runSetting.SetFileName(fileName);


        rng.seed(std::random_device{}());
        dynamicTimeout = dynTimeout;
        dynamicReallocation = dynReal;
        for(int i=0;i<numberOfRobots;i++)
        {
            Robot r{};

            r.id = i;
            r.state = RobotState::IDLE;
            r.expectedTravelTime = 0;
            r.actualTravelTime = 0;
            r.remainingTime = 0;
            r.averageTravelTime = 0;
            r.tasksCompleted = 0;
            r.availableAt = 0;
            r.recoveryAt = 0;

            robots.push_back(r);
        }
    }


    void updateRobotStates()
    {
        for(auto& robot : robots)
        {
            if(robot.state == RobotState::BUSY && robot.availableAt <= simulationTime)
            {
                robot.state = RobotState::IDLE;
            }
            if(robot.state == RobotState::FAILED && robot.recoveryAt <= simulationTime)
            {
                robot.state = RobotState::IDLE;
            }
        }
    }


    Robot* findAvailableRobot()
    {
        for(auto& robot : robots)
        {
            if(robot.state == RobotState::IDLE && robot.availableAt <= simulationTime) // first best robot:
            {
                return &robot;
            }
        }

        return nullptr;
    }

    Robot* findBestCandidate()
    {
        Robot* bestRobot = nullptr;

        for(auto& robot : robots)
        {
            if(robot.state != RobotState::IDLE)
                continue;

            if(bestRobot == nullptr || robot.averageTravelTime < bestRobot->averageTravelTime)
            {
                bestRobot = &robot;
            }
        }
        return bestRobot;
    }


    static double calculateDynamicTimeout(const double travelTime)
    {
        return travelTime * 1.5;
    }

    static void updateRobotTimes(Robot* robot, const double travelTime) {
        robot->averageTravelTime = (robot->averageTravelTime * robot->tasksCompleted + travelTime) / (robot->tasksCompleted + 1);
        robot->tasksCompleted++;
    }

    void simulateTask(const int taskId)
    {
        updateRobotStates();
        // auto failureReason = FailureReason::NONE;
        Robot* robot = findAvailableRobot();
        const double creationTick = simulationTime;
        while(robot == nullptr)
        {
            simulationTime += 5.0;

            updateRobotStates();

            robot = findAvailableRobot();
        }
        const double taskStartTick = simulationTime;
        const double waitingTime = taskStartTick - creationTick;
        const int assignedRobot = robot->id;
        int completedRobot = -1;
        if(!robot)
        {
            return;
        }

        std::uniform_real_distribution<double> travelDistribution(50.0,300.0);
        std::normal_distribution<double> delay(1.0, 0.4);
        std::uniform_real_distribution<double> failureDistribution(0.0,1.0);


        double expectedTravelTime = travelDistribution(rng);
        double timeout;
        if (dynamicTimeout) {
            timeout = calculateDynamicTimeout(expectedTravelTime);
        }
        else {
            timeout = TIMEOUT_LENGTH;
        }

        double actualTravelTime = expectedTravelTime * std::max(0.5, delay(rng));

        robot->state = RobotState::BUSY;
        robot->expectedTravelTime = expectedTravelTime;
        robot->actualTravelTime = actualTravelTime;
        robot->availableAt = simulationTime + actualTravelTime;

        const bool robotFailed = failureDistribution(rng) < failureChance;

        double blockedTime = 0;
        bool timeoutOccurred = false;
        bool unnecessaryAbort = false;

        bool reallocated = false;

        double reallocationTick = 0;
        double recoveryTime = 0;

        if(robotFailed)
        {
            const double failureTick = simulationTime;
            robot->state = RobotState::FAILED;
            robot->recoveryAt = simulationTime + 120;
            Robot* replacement = nullptr;

            while(replacement == nullptr) // look for replacement
            {
                simulationTime += 5.0;
                updateRobotStates();
                if (dynamicReallocation) {
                    replacement = findBestCandidate();
                }
                else {
                    replacement = findAvailableRobot();
                }


            }

            if(replacement) // replacement found
            {

                reallocated = true;
                reallocationTick = simulationTime;
                recoveryTime = reallocationTick - failureTick;
                reallocatedTasks++;

                replacement->state = RobotState::BUSY;

                actualTravelTime *= 1.25;
                replacement->availableAt = simulationTime + actualTravelTime;


                replacement->state = RobotState::IDLE;
                completedRobot = replacement->id;
                updateRobotTimes(replacement, actualTravelTime);
            }
            else // no replacement found :c
            {
                // failureReason = FailureReason::NO_REPLACEMENT;
                timeoutOccurred = true;
                timeoutTasks++;
            }
        }
        else
        {
            if(actualTravelTime > timeout) // robot didnt fail but took to long anyways :/
            {
                // failureReason = FailureReason::TIMEOUT;
                timeoutOccurred = true;
                blockedTime = actualTravelTime - timeout;
                totalBlockageTime+= blockedTime;

                if (actualTravelTime < timeout * (1+abortMargin)) {
                    unnecessaryAbort = true;
                    unnecessaryAborts++;
                }

                timeoutTasks++;
            }
            else // wow, nothing went wrong
            {
                updateRobotTimes(robot, actualTravelTime);
                completedRobot = robot->id;
                completedTasks++;
            }
        }
        previousTravelTimes.push_back(actualTravelTime);
        averageTaskTime = std::accumulate(previousTravelTimes.begin(), previousTravelTimes.end(), 0.0)/ static_cast<double>(previousTravelTimes.size());
        logger.log(
            taskId,
            assignedRobot,
            completedRobot,
            expectedTravelTime,
            actualTravelTime,
            creationTick,
            taskStartTick,
            robot->availableAt,
            waitingTime,
            // failureReason,
            recoveryTime,
            blockedTime,
            timeoutOccurred,
            reallocated,
            reallocationTick,
            unnecessaryAbort,
            !timeoutOccurred,
            dynamicTimeout,
            dynamicReallocation
        );

        robot->state = RobotState::IDLE;
    }

    void runSimulation(int taskCount)
    {
        auto start = std::chrono::steady_clock::now();

        for(int i=1;i<=taskCount;i++)
        {
            if (i%(taskCount/10) == 0) {
                std::cout << "1/10e voltooid: " << i << std::endl;
            }
            simulateTask(i);

            simulationTime += TASK_ARRIVAL_INTERVAL;
        }


        averageBlockageTime = totalBlockageTime/timeoutTasks;
        auto end =
            std::chrono::steady_clock::now();

        std::cout << "\n=== RESULTATEN ===\n";

        std::cout
            << "Taken: "
            << taskCount
            << "\n";
        std::cout
            << "Robots: "
            << robots.size()
            << std::endl;
        std::cout
            << "Faalpercentage: "
            << failureChance*100 << "%"
            << std::endl;
        std::cout
            << "Voltooid: "
            << completedTasks
            << std::endl;

        std::cout
            << "Timeouts: "
            << timeoutTasks
            << std::endl;
        std::cout
            << "Nutteloze timeouts: "
            << unnecessaryAborts
            << std::endl;
        std::cout
            << "gemiddelde blokkage tijd: "
            << averageBlockageTime
            << std::endl;

        std::cout
            << "Herallocaties: "
            << reallocatedTasks
            << std::endl;

        std::cout
            << "Throughput: "
            << completedTasks/simulationTime
            << " Taken per tijdseenheid"
            << std::endl;

        std::cout
            << "Gemiddelde tijd per taak: "
            << averageTaskTime
            << std::endl;

        runSetting.StartWriting();
        runSetting.log(
            taskCount,
            static_cast<int>(robots.size()),
            failureChance,
            completedTasks,
            timeoutTasks,
            unnecessaryAborts,
            averageBlockageTime,
            reallocatedTasks,
            (completedTasks/simulationTime),
            averageTaskTime);

        std::cout
            << "CSV en scenario opgeslagen in csv bestand\n";

        std::cout
            << "Duur: "
            << duration_cast<std::chrono::milliseconds>(end - start).count()
            << " ms\n";
    }
};

int main()
{

    int numberofTasks;
    int numberOfRobots;

    std::cout << "Geef de hoeveelheid taken om te simuleren: ";
    std::cin >> numberofTasks;
    std::cout << "Geef de hoeveelheid robots in de robotvloot: ";
    std::cin >> numberOfRobots;

    FleetManager fleet1(5, false, false, "statisch_zonder_herallocatie");
    fleet1.runSimulation(std::min(1000000, numberofTasks));

    FleetManager fleet2(5, false, true, "statisch_met_herallocatie");
    fleet2.runSimulation(std::min(1000000, numberofTasks));

    FleetManager fleet3(5, true, false, "dynamisch_zonder_herallocatie");
    fleet3.runSimulation(std::min(1000000, numberofTasks));

    FleetManager fleet4(5, true, true, "dynamisch_met_herallocatie");
    fleet4.runSimulation(std::min(1000000, numberofTasks));

    return 0;
}