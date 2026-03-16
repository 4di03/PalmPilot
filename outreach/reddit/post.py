#!/usr/bin/env python3
"""
Reddit outreach poster.

Usage:
  python post.py --dry-run          # Preview all posts without submitting
  python post.py --niche food-truck # Post only one niche (matches filename prefix)
  python post.py                    # Post everything (15-min gap between posts)

Set credentials in .env before running. See .env.example.
"""

import praw
import json
import time
import os
import sys
import argparse
from pathlib import Path
from dotenv import load_dotenv

load_dotenv()

POSTS_DIR = Path(__file__).parent / "posts"
DELAY_BETWEEN_POSTS = 900  # 15 minutes — avoids spam filters

NICHE_FILE_MAP = {
    "food-truck":        "food_truck.json",
    "nonprofit-grants":  "nonprofit_grants.json",
    "machine-shop":      "machine_shop.json",
    "gun-shop":          "gun_shop.json",
    "cleaning-business": "cleaning_business.json",
    "ada-compliance":    "ada_compliance.json",
    "auto-detailing":    "auto_detailing.json",
    "martial-arts":      "martial_arts.json",
}


def load_posts(niche_filter=None):
    posts = []
    files = POSTS_DIR.glob("*.json")
    for f in sorted(files):
        if niche_filter:
            target = NICHE_FILE_MAP.get(niche_filter)
            if target and f.name != target:
                continue
        with open(f) as fp:
            data = json.load(fp)
            for post in data:
                post["_source_file"] = f.name
            posts.extend(data)
    return posts


def inject_url(posts, landing_url):
    """Replace LANDING_PAGE_URL placeholder with the actual deployed URL."""
    for p in posts:
        p["body"] = p["body"].replace("LANDING_PAGE_URL", landing_url)
    return posts


def dry_run(posts):
    print(f"\n{'─'*60}")
    print(f"DRY RUN — {len(posts)} post(s) queued. Nothing will be submitted.\n")
    for i, p in enumerate(posts, 1):
        print(f"[{i}/{len(posts)}] r/{p['subreddit']}  ({p['_source_file']})")
        print(f"TITLE: {p['title']}")
        print(f"BODY:\n{p['body']}")
        print(f"{'─'*60}\n")


def get_reddit_client():
    required = ["REDDIT_CLIENT_ID", "REDDIT_CLIENT_SECRET", "REDDIT_USERNAME", "REDDIT_PASSWORD"]
    missing = [k for k in required if not os.getenv(k)]
    if missing:
        print(f"Missing .env values: {', '.join(missing)}")
        print("Copy .env.example → .env and fill in your credentials.")
        sys.exit(1)

    return praw.Reddit(
        client_id=os.getenv("REDDIT_CLIENT_ID"),
        client_secret=os.getenv("REDDIT_CLIENT_SECRET"),
        username=os.getenv("REDDIT_USERNAME"),
        password=os.getenv("REDDIT_PASSWORD"),
        user_agent=f"script:interest_validator:v1 (by u/{os.getenv('REDDIT_USERNAME')})",
    )


def post_all(posts, reddit, delay=DELAY_BETWEEN_POSTS):
    results = []
    for i, p in enumerate(posts, 1):
        try:
            sub = reddit.subreddit(p["subreddit"])
            submission = sub.submit(title=p["title"], selftext=p["body"])
            url = f"https://reddit.com{submission.permalink}"
            print(f"[{i}/{len(posts)}] ✓ Posted to r/{p['subreddit']}")
            print(f"         {url}")
            results.append({"subreddit": p["subreddit"], "url": url, "status": "ok"})
        except Exception as e:
            print(f"[{i}/{len(posts)}] ✗ Failed r/{p['subreddit']}: {e}")
            results.append({"subreddit": p["subreddit"], "url": None, "status": str(e)})

        if i < len(posts):
            mins = delay // 60
            print(f"         Waiting {mins}m before next post...\n")
            time.sleep(delay)

    print(f"\nDone. {sum(1 for r in results if r['status'] == 'ok')}/{len(results)} posted successfully.")
    return results


def main():
    parser = argparse.ArgumentParser(description="Post to Reddit communities.")
    parser.add_argument("--dry-run", action="store_true", help="Preview posts without submitting")
    parser.add_argument("--niche", help="Only post for one niche (e.g. food-truck)")
    parser.add_argument("--url", default="LANDING_PAGE_URL", help="Landing page URL to embed in posts")
    args = parser.parse_args()

    posts = load_posts(niche_filter=args.niche)

    if not posts:
        print("No posts found. Check --niche matches a known niche name.")
        sys.exit(1)

    posts = inject_url(posts, args.url)

    if args.dry_run:
        dry_run(posts)
        return

    print(f"\nAbout to submit {len(posts)} post(s) to Reddit.")
    print("Posts will be spaced 15 minutes apart to avoid spam filters.")
    print("Press Ctrl+C to cancel.\n")
    time.sleep(4)

    reddit = get_reddit_client()
    post_all(posts, reddit)


if __name__ == "__main__":
    main()
