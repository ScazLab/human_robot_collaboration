# Contributing

We **love** pull requests from _everyone_. Third-party contributions are essential for keeping this repository great.

## Making Changes

Fork, then clone the repo:

```git clone git@github.com:your-username/human_robot_collaboration.git```

Make sure you can [compile the code, run the code, run the tests][compilation].

Create a branch from where you want to base your work (usually, the `master` branch). E.g.:

```
cd human_robot_collaboration
git checkout master
git checkout -b feature/my_feature
```

Please try to give some meaningful name to your branch. We usually use the format `type_of_contribution`+`/`+`actual_contribution` (e.g. `fix/gripper_bug` or `test/better_gripper_testing`).

Make commits of logical units with ideally a a [good commit message][commit]. Write good documentation. Be sure to follow the [style guide][style_guide]. Any contribution will NOT be accepted if it does not follow the style guide.

Add tests for your change. Make the tests pass.

Push to your fork and [submit a pull request][pr].

[compilation]: https://github.com/ScazLab/human_robot_collaboration#compilation--testing
[style_guide]: https://github.com/ScazLab/human_robot_collaboration/blob/master/STYLE_GUIDE.md
[pr]: https://github.com/scazlab/human_robot_collaboration/compare/
[commit]: http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html
