# MIT License

# Copyright (c) 2024 Saurabh Gupta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
from pathlib import Path

import typer

app = typer.Typer(add_completion=False, rich_markup_mode="rich")


@app.command()
def rosbag_reader(
    rosbag_path: Path = typer.Argument(None, show_default=False, help="Path to the Rosbag File"),
    topic: str = typer.Argument(
        None, show_default=False, case_sensitive=True, help="Name of the topic to read"
    ),
    output_path: Path = typer.Argument(
        None, show_default=False, help="Path to store the parsed data"
    ),
):
    from rosbag_reader.rosbag_reader import RosbagReader

    rosbag_reader = RosbagReader(rosbag_path.as_posix())
    rosbag_reader.printInfo()
    rosbag_reader.saveDataOnTopic(topic, output_path.as_posix())


def run():
    app()
