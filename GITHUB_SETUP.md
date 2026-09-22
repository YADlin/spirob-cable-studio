# Put this repository on GitHub

The delivered `spirob-cable-studio` folder is already a local Git repository on `main`, with the source, tests, documentation, pinned dependencies and a simulator-only GitHub Actions workflow committed. No remote is configured. The commit is attributed to the software assistant; configure your own identity before making future commits.

## 1. Extract and check the repository

```bash
cd ~/Two_Cable_Motor_Setup
unzip ~/Downloads/spirob-cable-studio.zip
cd spirob-cable-studio
git status
git log -1 --oneline
```

`git status` should show branch `main`. Local `config.json`, logs and virtual environments are ignored. A typical session does not dirty tracked source files.

## 2. Authenticate with GitHub

If `gh` is already installed:

```bash
gh auth login
gh auth status
```

Choose GitHub.com and the account where the research repository should live. Do not paste access tokens into chat or source files. If the `gh` command is unavailable on Ubuntu, install it using your package manager or the [official GitHub CLI installation instructions](https://github.com/cli/cli/blob/trunk/docs/install_linux.md).

## 3. Create a private repository and push the committed source

Run from the project directory:

```bash
gh repo create spirob-cable-studio --private --source=. --remote=origin --push
gh repo view --web
```

This creates the repository under the authenticated account. It does not publish your ignored local configuration or logs. If the name already exists, choose a different name in `gh repo create`; do not force-push over an existing research repository.

The command follows [GitHub's repository-creation CLI documentation](https://cli.github.com/manual/gh_repo_create). Private visibility is used for the initial research repository; changing visibility later is a separate deliberate decision.

## 4. Inspect the automated check

Open the repository's **Actions** tab. The workflow installs the pinned packages and runs the simulator/protocol/Qt tests with an offscreen Qt platform. It never connects to your motor hardware. Local tests passed during development; the first remote Actions result still needs to be checked after publication.

For later changes:

```bash
git status
git add spirob_cable tests README.md OPERATIONS.md ENGINEERING.md
git commit -m "Describe the research change"
git push
```

Stage the files you intentionally changed. The application already runs from the repository folder; you do not need another ZIP for each future code change once Git is your workflow.
