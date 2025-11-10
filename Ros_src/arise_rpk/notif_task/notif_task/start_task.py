
import rclpy

from rclpy.executors import MultiThreadedExecutor

import notif_task.task_impl


def main():
    rclpy.init()

    task = notif_task.task_impl.TaskImpl()
    task_executor = MultiThreadedExecutor()
    task_executor.add_node(task)

    try:
        task_executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Goodbye!")
        task.destroy_node()


if __name__ == '__main__':
    main()
