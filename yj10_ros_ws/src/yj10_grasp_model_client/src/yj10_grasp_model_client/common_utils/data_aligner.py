from collections import deque
from typing import Set, Dict, Deque, Optional, Tuple
from dataclasses import dataclass
import logging

kLogger = logging.getLogger("DataAligner")


@dataclass
class DataTimed:
    timestamp: float
    data: any


@dataclass
class AlignedDataCollection:
    timestamp: Optional[float]
    data_dict: Dict[str, Optional[DataTimed]]


def find_nearest_data(data_queue: Deque[DataTimed], timestamp: float) -> Tuple[DataTimed, int, float]:
    """Find the index of the data with timestamp closest to the given timestamp

    Returns:
        Tuple[DataTimed, int, float]: data, index, absolute difference in timestamps
    """
    min_diff = float("inf")
    min_i = None

    for i, data_timed in enumerate(data_queue):
        diff = abs(data_timed.timestamp - timestamp)
        if diff == 0:
            return data_timed, i, diff

        if diff < min_diff:
            min_diff = diff
            min_i = i

    return data_queue[min_i], min_i, min_diff


class DataAligner:
    def __init__(self, data_configs: Set[dict], reference_data: str, callback, timestamp_tolerance: float = 0.0):
        """Class to align data from multiple sources based on timestamps.
        This class will store data in queues and align them to the reference data. It maintains a candidate result that is the best so far.
        The candidate result will be updated when new data added by add_data() is better.
        The callback function will be called when the candidate result are aligned.

        The candidate result are considered aligned if they are optimal or within the timestamp_tolerance.
        Note:
            1.  When the candidate result is optimal, it may not be within the timestamp_tolerance.
                So the callback may give you data that is not within the timestamp_tolerance if impossible.
            2.  If timestamp_tolerance is too small, the callback won't be called in time
                because this class need to evaluate new data to check if the candidate result is optimal.

        Args:
            data_configs (Set[dict]): The dict keys accepted are:
                name (str, required): Data name.
                maxlen (int, optional): Max length of the queue to store data, optional. The default is 50.

            reference_data (str): Data to align to.
            callback (function): Function to call when all data are aligned.
            timestamp_tolerance (float): Data within this tolerance will be considered aligned.
        """
        super().__init__()

        self.reference_data = reference_data

        self.data_queue_dict: Dict[str, Deque[DataTimed]] = dict()
        self.candidate_result: AlignedDataCollection = AlignedDataCollection(timestamp=None, data_dict=dict())

        for config in data_configs:
            name = config["name"]
            self.data_queue_dict[name] = deque(maxlen=config.get("maxlen", 50))
            self.candidate_result.data_dict[name] = DataTimed(timestamp=None, data=None)

        self.callback = callback  # function to call when all data are available
        self.timestamp_tolerance = timestamp_tolerance

    def add_data(self, name: str, data, timestamp: Optional[float] = None):
        """
        Args:
            name (str): Data name
            data : data
            timestamp (Optional[float]): The timestamp of the data. If None, it will try to get the timestamp from the data.
        """

        assert (
            name in self.data_queue_dict
        ), f"Data name '{name}' not in the list of data names: {self.data_queue_dict.keys()}"

        if timestamp is None:
            timestamp = self.get_timestamp(data)

        if not self.__check_if_timestamp_is_increasing(self.data_queue_dict[name], timestamp, name):
            kLogger.warning(f"Ignoring {name}")
            return

        if name == self.reference_data:
            self.__add_reference_data(data, timestamp)
        else:
            self.__add_other_data(name, data, timestamp)

        if self.is_candidate_result_within_tolerance() or self.is_candidate_result_optimal():
            self.callback(self.candidate_result)
            if len(self.data_queue_dict[self.reference_data]) == 0:
                self.candidate_result.timestamp = None
                self.candidate_result.data_dict[self.reference_data] = DataTimed(timestamp=None, data=None)
            else:
                self.__switch_to_next_reference_data()

    def get_now_candidate_result(self) -> Optional[AlignedDataCollection]:
        """Get the best result so far, but may not be the optimal result.
        If no more data is coming, you can use this result to avoid the last reference data being dropped.

        Returns:
            Optional[AlignedDataCollection]: The best result so far if there is reference data in this class. Otherwise, None.
        """
        if self.candidate_result.timestamp is not None:
            return self.candidate_result
        return None

    def __add_reference_data(self, data, timestamp: float) -> bool:
        """Add reference data to candidate result or queue

        Returns:
            bool: True if the candidate result is updated. False if the data is added to the queue
        """
        if self.candidate_result.timestamp is None:
            self.candidate_result.timestamp = timestamp
            self.candidate_result.data_dict[self.reference_data] = DataTimed(timestamp=timestamp, data=data)
            self.__update_candidate_result_with_all_other_data()
            return True
        else:
            self.data_queue_dict[self.reference_data].append(DataTimed(timestamp=timestamp, data=data))
            return False

    def __add_other_data(self, name: str, data, timestamp: float) -> bool:
        """Add non-reference data to candidate result or queue

        Returns:
            bool: True if the candidate result is updated. False if the data is added to the queue
        """
        if len(self.data_queue_dict[name]) == 0:
            # No data in the queue, this data may be better than the one in the candidate result
            is_updated = self.__update_candidate_result(name, DataTimed(timestamp=timestamp, data=data))
            if is_updated:
                return True

        # This data is not better than the one in the candidate result because:
        # 1. The queue is not empty
        # 2. The queue is empty but is_updated == False
        self.data_queue_dict[name].append(DataTimed(timestamp=timestamp, data=data))

        return False

    def __update_candidate_result_with_all_other_data(self):
        for name, data_queue in self.data_queue_dict.items():
            if name == self.reference_data:
                continue

            while len(data_queue) > 0:
                data = data_queue[0]
                is_updated = self.__update_candidate_result(name, data)
                if is_updated:
                    data_queue.popleft()
                else:
                    break

    def __switch_to_next_reference_data(self):
        next_data = self.data_queue_dict[self.reference_data].popleft()
        self.candidate_result.timestamp = next_data.timestamp
        self.candidate_result.data_dict[self.reference_data] = next_data
        self.__update_candidate_result_with_all_other_data()

    def get_timestamp(self, msg) -> float:
        """Get timestamp from ros message with header"""
        return msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9

    def __update_candidate_result(self, name: str, data: DataTimed) -> bool:
        """Update the candidate result if the new data is closer to the reference timestamp

        Args:
            name (str): Data name
            data (DataTimed): Data to compare

        Returns:
            bool: True if the candidate result is updated
        """

        original_timestamp = self.candidate_result.data_dict[name].timestamp
        if original_timestamp is None:
            self.candidate_result.data_dict[name] = data
            return True

        reference_timestamp = self.candidate_result.timestamp
        if reference_timestamp is None:
            return False

        original_diff = abs(reference_timestamp - original_timestamp)
        new_diff = abs(reference_timestamp - data.timestamp)

        if new_diff <= original_diff:
            self.candidate_result.data_dict[name] = data
            return True

        return False

    def is_candidate_result_optimal(self) -> bool:
        """Check if the candidate result is optimal"""
        if self.candidate_result.timestamp is None:
            return False

        for name, data_queue in self.data_queue_dict.items():
            if name == self.reference_data:
                continue

            if len(data_queue) == 0:
                # If the queue is empty, we don't know if new data would be better.
                return False

            # If data_queue is not empty, we know:
            # 1. data_queue[0] is not better than candidate (candidate = candidate_result.data_dict[name])
            # 2. data_queue[0].timestamp > reference_data's timestamp
            # 3. data_queue[1] is not better than data_queue[0], because the timestamp is increasing
            # 4. Advantages: candidate > data_queue > new comming data

        # If all data_queue are not empty, the candidate result is optimal
        return True

    def is_candidate_result_within_tolerance(self) -> bool:
        """Check if the candidate result is within the timestamp tolerance"""
        reference_timestamp = self.candidate_result.timestamp
        if reference_timestamp is None:
            return False

        for name, data_timed in self.candidate_result.data_dict.items():
            if name == self.reference_data:
                continue

            if data_timed.timestamp is None:
                return False

            if abs(data_timed.timestamp - reference_timestamp) > self.timestamp_tolerance:
                return False

        return True

    def __check_if_timestamp_is_increasing(
        self, data_queue: Deque[DataTimed], new_timestamp: float, name: Optional[str] = None
    ) -> bool:
        if len(data_queue) > 0:
            last_timestamp = data_queue[-1].timestamp
        else:
            last_timestamp = self.candidate_result.data_dict[name].timestamp

        if last_timestamp is None or new_timestamp >= last_timestamp:
            return True
        else:
            if name is not None:
                kLogger.warning(f"Timestamp of {name} is not increasing: {last_timestamp} -> {new_timestamp}")
            else:
                kLogger.warning(f"Timestamp is not increasing: {last_timestamp} -> {new_timestamp}")

            return False


if __name__ == "__main__":

    def callback(data):
        print(data)

    data_configs = [
        {"name": "data1", "maxlen": 50},
        {"name": "data2", "maxlen": 50},
        {"name": "data3", "maxlen": 50},
    ]

    data_aligner = DataAligner(data_configs, "data1", callback, 0.06)
    data_aligner.add_data("data1", 1, timestamp=1.0)
    data_aligner.add_data("data2", 1, timestamp=1.0)
    data_aligner.add_data("data3", 1, timestamp=1.0)

    data_aligner.add_data("data2", 2, timestamp=1.9)
    data_aligner.add_data("data3", 2, timestamp=1.95)
    data_aligner.add_data("data1", 2, timestamp=2.0)
    data_aligner.add_data("data2", 2, timestamp=2.05)

    data_aligner.add_data("data1", 3, timestamp=3.0)
    data_aligner.add_data("data1", 4, timestamp=4.0)
    data_aligner.add_data("data2", 3, timestamp=3.0)
    data_aligner.add_data("data3", 3, timestamp=3.0)

    data_aligner.add_data("data2", 4, timestamp=3.8)
    data_aligner.add_data("data3", 4, timestamp=4.0)
    print(f"now_candidate_result: {data_aligner.get_now_candidate_result()}")

    data_aligner.add_data("data2", 4, timestamp=3.7)
    data_aligner.add_data("data2", 4, timestamp=3.6)

    print("Done!")
    # pass
