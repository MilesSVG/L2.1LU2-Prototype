#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <random>
#include <algorithm>


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

// enum class FailureReason {
//     NONE,
//     TIMEOUT,
//     NO_REPLACEMENT
// };


//will save various values that are relevant to each task
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
        const int taskId,
        const int assignedRobotId,
        const int completedRobotId,
        const double expectedTravelTime,
        const double actualTravelTime,
        const double creationTick,
        const double startTick,
        const double finishTick,
        const double waitingTime,
        const double recoveryTime,
        const double blockageTime,
        const bool timeout,
        const bool reallocated,
        const double reallocationTick,
        const bool unnecessaryAbort,
        const bool completed,
        const bool dynamicTimeout,
        const bool dynamicReallocation)
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

//will save various values relevant to the entire run
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
        const int tasks,
        const int robots,
        const double failureRate,
        const int completed,
        const int timeouts,
        const int unnecessaryTimeouts,
        const double averageBlockageTime,
        const int reallocations,
        const double throughput,
        const double averageTaskTime)
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

        //initialises all the robots in the fleet
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

    //will set robots idle if they should be available (available time <= simulation time)
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

    //will look for the first idle robot it can find
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

    //will look for the robot with the shortest average travel time
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

    //bases the timeout value on each task
    static double calculateDynamicTimeout(const double travelTime)
    {
        return travelTime * 1.5;
    }

    //updates a robots average travel time and completed task counter
    static void updateRobotTimes(Robot* robot, const double travelTime) {
        robot->averageTravelTime = (robot->averageTravelTime * robot->tasksCompleted + travelTime) / (robot->tasksCompleted + 1);
        robot->tasksCompleted++;
    }

    //will simulate a task being performed
    void simulateTask(const int taskId)
    {
        updateRobotStates();
        // auto failureReason = FailureReason::NONE;
        Robot* robot = findAvailableRobot();
        const double creationTick = simulationTime;

        //tries to find an available robot, ticks the simulation forward if none are found
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

        //defines the tasks timeout length based on the dynamicTimeout setting
        if (dynamicTimeout) {
            timeout = calculateDynamicTimeout(expectedTravelTime);
        }
        else {
            timeout = TIMEOUT_LENGTH;
        }

        //how long the task actually takes differs from what the simulation thinks, this allows timeouts to occur
        double actualTravelTime = expectedTravelTime * std::max(0.5, delay(rng));

        robot->state = RobotState::BUSY;
        robot->expectedTravelTime = expectedTravelTime;
        robot->actualTravelTime = actualTravelTime;
        robot->availableAt = simulationTime + actualTravelTime;

        //produces a number between 0.0 and 1 and will have a failureChance% chance at simulating a robot failing
        const bool robotFailed = failureDistribution(rng) < failureChance;

        double blockedTime = 0;
        bool timeoutOccurred = false;
        bool unnecessaryAbort = false;

        bool reallocated = false;

        double reallocationTick = 0;
        double recoveryTime = 0;

        //tries to find a robot to reallocate the task to
        if(robotFailed)
        {
            const double failureTick = simulationTime;
            robot->state = RobotState::FAILED;
            robot->recoveryAt = simulationTime + 120;
            Robot* replacement = nullptr;

            //ticks the simulation while a suitable replacement cant be found
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

                //considers how much time it wouldve taken for the task to be completed anyways, then decides if the timeout is unnecessary based on the abortMargin
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
        //calculates final values and adds to the logs
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
        //robot is done with its task now, it'll be set to idle
        robot->state = RobotState::IDLE;
    }

    //simulates n tasks and logs the runs results
    void runSimulation(const int taskCount)
    {
        const auto start = std::chrono::steady_clock::now();


        for(int i=1;i<=taskCount;i++)
        {
            if (i%(taskCount/10) == 0) {
                std::cout << "1/10e voltooid: " << i << std::endl;
            }
            simulateTask(i);

            simulationTime += TASK_ARRIVAL_INTERVAL;
        }


        averageBlockageTime = totalBlockageTime/timeoutTasks;
        const auto end = std::chrono::steady_clock::now();

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

    int numberOfTasks;
    int numberOfRobots;

    //not idiot proof, dont try entering invalid values
    std::cout << "Geef de hoeveelheid taken om te simuleren: ";
    std::cin >> numberOfTasks;
    std::cout << "Geef de hoeveelheid robots in de robotvloot: ";
    std::cin >> numberOfRobots;

    //simulates each possible iteration of research relevant settings
    FleetManager fleet1(numberOfRobots, false, false, "statisch_zonder_herallocatie");
    fleet1.runSimulation(std::min(1000000, numberOfTasks));

    FleetManager fleet2(numberOfRobots, false, true, "statisch_met_herallocatie");
    fleet2.runSimulation(std::min(1000000, numberOfTasks));

    FleetManager fleet3(numberOfRobots, true, false, "dynamisch_zonder_herallocatie");
    fleet3.runSimulation(std::min(1000000, numberOfTasks));

    FleetManager fleet4(numberOfRobots, true, true, "dynamisch_met_herallocatie");
    fleet4.runSimulation(std::min(1000000, numberOfTasks));

    return 0;
}